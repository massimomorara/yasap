
#if 0

YASAP 0.1.0 - Yet Another Simple Audio Player

Copyright (c) 2018 massimo morara

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Except as contained in this notice, the names of the authors or above
copyright holders shall not be used in advertising or otherwise to promote
the sale, use or other dealings in this Software without prior written
authorization.

#endif

#include <array>
#include <mutex>
#include <queue>
#include <atomic>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <condition_variable>

extern "C"
 {
   #include <SDL2/SDL.h>
   #include <libavformat/avformat.h>
   #include <libswresample/swresample.h>
 }

#include <unistd.h>
#include <term.h>


template <typename T, std::size_t maxSize = 16U>
class cBuffer
 {
   private:
      std::mutex               m;
      std::deque<T>            q;
      std::condition_variable  cvNf; // not full
      std::condition_variable  cvNe; // not empty

   public:
      cBuffer ()                            = default;
      cBuffer (cBuffer const &)             = delete;
      cBuffer (cBuffer &&)                  = delete;
      cBuffer & operator= (cBuffer const &) = delete;
      cBuffer & operator= (cBuffer &&)      = delete;

      void purge ()
       {
         std::unique_lock<decltype(m)> l{m};

         q.clear();

         cvNf.notify_one();
       }

      template <typename TimeType>
      bool try_pop (T & t, TimeType const & d)
       {
         std::unique_lock<decltype(m)> l{m};

         bool const b { cvNe.wait_for(l, d,
                           [this]{ return false == q.empty(); }) };

         if ( b )
          {
            t = std::move(q.front());

            q.pop_front();

            cvNf.notify_one();
          }

         return b;
       }

      template <typename ... Args>
      void push (Args && ... as)
       {
         std::unique_lock<decltype(m)> l{m};

         cvNf.wait(l, [this]{ return q.size() <= maxSize; });

         q.emplace_back(std::forward<Args>(as)...);

         cvNe.notify_one();
       }
 };

void sdl_audio_callback (void *, std::uint8_t *, int);

struct audioData
 {
   // static data 
   static std::unordered_map<AVSampleFormat, int> const formM;
   static constexpr AVSampleFormat defFfmpegFormat { AV_SAMPLE_FMT_S16 };
   static constexpr int defSdlFormat { AUDIO_S16SYS };
   static constexpr int maxOutFrames { 1 << 16 };

   // ffmpeg data
   AVFormatContext *  frmCtx   { nullptr };
   AVCodecContext  *  codecCtx { nullptr };
   SwrContext *       swrCtx   { nullptr };
   std::uint8_t *     swrBuff  { nullptr };
   int                aStreamI { -1 }; 

   // sdl data
   SDL_AudioSpec      aSpc  {};
   SDL_AudioDeviceID  devId {};

   // cBuffer data
   using buffData = std::pair<std::vector<std::uint8_t>, std::int64_t>;

   cBuffer<buffData>          cb;
   buffData                   bd;
   std::size_t                vp      { 0U };
   std::size_t                oldPosS = -1;
   std::atomic<std::int64_t>  ptrkPos { 0 };  // prev track starting pos
   std::int64_t               ttrkPos { 0 };  // this track starting pos
   std::atomic<std::int64_t>  ntrkPos { 0 };  // next track starting pos
   std::atomic<bool>          aPurged { false };
   std::atomic<std::int64_t>  aPos    { 0 };

   // time position / time base data
   std::size_t   lenF, lenH, lenM, lenS;
   std::int64_t  tbNum, tbDen;

   // tracks data (if any)
   std::vector<std::pair<std::int64_t, std::string>>  tracksData;
   std::size_t                                        numTracks  { 0U };

   // other data
   std::atomic<bool> toQuit   { false };  // is required to quit now?
   bool              frmConv  { false };  // is format conversion needed?

   audioData ()
    { };

   static std::string getFileName (std::string const & str)
    {
      std::string ret;

      for ( auto const & ch : str )
         if ( (ch == '\\') || (ch == '/' ) )
             ret.clear();
         else
             ret.push_back(ch);

      return ret;
    }

   static std::string toLower (std::string str)
    {
      static constexpr char diffCase { 'a' - 'A' };

      for ( auto & ch : str )
         if ( (ch >= 'A') && (ch <= 'Z') )
            ch += diffCase;

      return str;
    }

   void addTrack (std::string const & writ, std::string const & perf,
                  std::string const & titl, std::int64_t strt)
    {
      std::string  str;

      if ( (false == writ.empty()) && (false == perf.empty()) )
         str = writ + " (perf. " + perf + ")";
      else
         str = writ + perf;

      if ( false == str.empty() )
         str += " - ";

      if ( titl.empty() )
         str += "track " + std::to_string(tracksData.size()+1U);
      else
         str += titl;

      tracksData.emplace_back(strt, std::move(str));
    }

   void setTracksData (std::string const & str)
    {
      bool                tracksZone{false}, indexed{false};
      std::int64_t        strt{0};
      std::string         line;
      std::string         cueCommand;
      std::string         writ, perf, titl;
      std::istringstream  issf{str};

      while ( std::getline(issf, line) )
       {
         std::istringstream  issl{line};

         if ( issl >> cueCommand )
          {
            auto const lowC { toLower(cueCommand) };

            if ( "track" == lowC )
             {
               if ( tracksZone )
                  addTrack(writ, perf, titl, strt);
               else
                  tracksZone = true;

               indexed = false;
               strt    = 0;

               writ.clear();
               perf.clear();
               titl.clear();
             }
            else if ( false == tracksZone )
               ;
            else if ( "songwriter" == lowC )
               issl >> std::quoted(writ);
            else if ( "performer" == lowC )
               issl >> std::quoted(perf);
            else if ( "title" == lowC )
               issl >> std::quoted(titl);
            else if ( ("index" == lowC) && (false == indexed) )
             {
               indexed = true;

               char  ch1, ch2;
               int   numI, mm, ss, ff;

               issl >> numI >> mm >> ch1 >> ss >> ch2 >> ff;

               strt = ((mm*60.0 + ss + ff/75.0) * tbDen) / tbNum;
             }
          }
       }

      if ( tracksZone )
       {
         // add last track data
         addTrack(writ, perf, titl, strt);

         // memorize num of tracks
         numTracks = tracksData.size();
       }
    }

   void initialize (std::string const & fileName)
    {
      using namespace std::string_literals;

      // initialize ffmpeg

      av_register_all();

      if ( 0 > avformat_open_input(&frmCtx, fileName.c_str(), nullptr,
                                   nullptr) )
         throw std::runtime_error("error opening "s + fileName);

      if ( 0 > avformat_find_stream_info(frmCtx, nullptr) )
         throw std::runtime_error("no stream information in file");

      for ( auto ui = 0U;    (-1 == aStreamI)
                          && (ui < frmCtx->nb_streams) ; ++ui )
         if (   AVMEDIA_TYPE_AUDIO
             == frmCtx->streams[ui]->codecpar->codec_type )
            aStreamI = ui;

      if ( -1 == aStreamI )
         throw std::runtime_error("no audio stream in file");

      tbNum = frmCtx->streams[aStreamI]->time_base.num;
      tbDen = frmCtx->streams[aStreamI]->time_base.den;

      lenF  = frmCtx->duration / AV_TIME_BASE;
      lenH  = lenF / 3600U;
      lenM  = (lenF % 3600U) / 60U;
      lenS  = lenF % 60U;

      AVCodec * codec = avcodec_find_decoder(
         frmCtx->streams[aStreamI]->codecpar->codec_id);

      if ( nullptr == codec )
         throw std::runtime_error("unsupported audio codec");

      if ( nullptr == (codecCtx = avcodec_alloc_context3(codec)) )
         throw std::runtime_error("error allocating context");

      if ( 0 > avcodec_parameters_to_context
          (codecCtx, frmCtx->streams[aStreamI]->codecpar) )
         throw std::runtime_error("error copying codec context");

      if ( 0 > avcodec_open2(codecCtx, codec, nullptr) )
         throw std::runtime_error("error opening audio codec");

      // initialize sdl

      if ( SDL_Init(SDL_INIT_AUDIO | SDL_INIT_TIMER) )
         throw std::runtime_error(  "error initializing SDL: "s
                                  + SDL_GetError());

      char const * nmPtr { av_get_sample_fmt_name(codecCtx->sample_fmt) };

      std::cout << '\n'
         << "- filename:      " << getFileName(fileName) << '\n'
         << "- audio codec:   " << codec->long_name << '\n'
         << "- sample format: " << (nmPtr ? nmPtr : "unrecognized") << '\n'
         << "- # of channels: " << codecCtx->channels << '\n'
         << "- sample rate:   " << codecCtx->sample_rate << " Hz\n";

      if ( frmCtx->metadata )
       {
         AVDictionaryEntry * mdp { nullptr };

         std::cout << "\n--- metadata:\n";

         while ( nullptr != (mdp = av_dict_get(frmCtx->metadata, "", mdp,
                                             AV_DICT_IGNORE_SUFFIX)) )
          {
            auto const key { toLower(mdp->key) };

            if ( "cuesheet" == key )
               setTracksData(mdp->value);
            else
               std::cout << " - " << key << ": " << mdp->value << '\n';
          }

         if ( numTracks )
          {
            std::cout << "\n--- tracks\n";

            for ( auto ui = 0U ; ui < numTracks ; ++ui )
               writeTracks(ui, ' ');
          }
       }

      std::cout << std::endl;

      auto const iFrm { formM.find(codecCtx->sample_fmt) };

      SDL_AudioSpec req;

      req.freq     = codecCtx->sample_rate;
      req.format   = ( iFrm != formM.cend()
                      ? iFrm->second
                      : (frmConv = true, defSdlFormat) );
      req.channels = codecCtx->channels;
      req.silence  = 0;
      req.samples  = 1024;
      req.callback = sdl_audio_callback;
      req.userdata = static_cast<void *>(this);

      if ( 0 == (devId = SDL_OpenAudioDevice(nullptr, 0, &req, &aSpc,
                                             SDL_AUDIO_ALLOW_ANY_CHANGE)) )
         throw std::runtime_error("error opening SDL");

      if (   (req.freq     != aSpc.freq)
          || (req.format   != aSpc.format)
          || (req.channels != aSpc.channels) )
         throw std::runtime_error("SDL specs device error");
      
      if ( true == frmConv )
       {
         if ( nullptr == (swrCtx
            = swr_alloc_set_opts(nullptr, codecCtx->channel_layout,
                                 defFfmpegFormat, codecCtx->sample_rate,
                                 codecCtx->channel_layout,
                                 codecCtx->sample_fmt,
                                 codecCtx->sample_rate, 0, nullptr) ) )
            throw std::runtime_error("error setting swr context");

         if ( 0 > swr_init(swrCtx) )
            throw std::runtime_error("error initializing swr context");

         if ( 0 > av_samples_alloc(&swrBuff, nullptr, aSpc.channels,
                                   maxOutFrames, defFfmpegFormat, 0) )
            throw std::runtime_error("error allocating swr buffer");
       }
    }

   template <typename T>
   void writeTracks (std::size_t nTrk, T const & pre)
    {
      if ( nTrk < numTracks )
         std::cout << pre << std::setfill(' ')
            << std::setw(1+(9U<numTracks)) << (nTrk+1U) << " - "
            << tracksData[nTrk].second << '\n';
    }

   void changeTrack (std::int64_t pos)
    {
      auto trk { numTracks };

      ptrkPos = (numTracks > 1U ? tracksData[numTracks-2U].first : 0);
      ttrkPos = tracksData.back().first;
      ntrkPos = frmCtx->duration;

      for ( auto ui { 1U } ; ui < trk ; ++ui )
         if ( tracksData[ui].first >= pos )
          {
            trk     = ui;
            ptrkPos = trk > 1U ? tracksData[trk-2U].first : 0;
            ttrkPos = tracksData[trk-1U].first;
            ntrkPos = tracksData[trk].first;
          }

      std::cout << "\n\n";

      writeTracks(trk-2U, "      ");
      writeTracks(trk-1U, " >>>  ");
      writeTracks(trk   , "      ");

      std::cout << '\n';
    }

   void sdl_acb (std::uint8_t * stream, int len)
    {
      if ( len < 0 )
         throw std::runtime_error("negative request in sdl_acb");

      std::size_t uLen = len;

      std::uint8_t * const cpStream { stream };

      if ( aPurged.exchange(false) )
       {
         bd.first.clear();

         vp = 0U;
       }

      bool  loop;

      do
       {
         auto const l { std::min(uLen, bd.first.size()-vp) };

         if ( l )
          {
            std::copy(bd.first.data()+vp, bd.first.data()+vp+l, stream);

            stream += l;
            uLen   -= l;
            vp     += l;
          }

         if ( uLen )
          {
            using namespace std::chrono_literals;

            loop = cb.try_pop(bd, 10ms);
            vp   = 0U;

            if ( false == loop )
               bd.first.clear();
            else
             {
               aPos = bd.second;

               if ( numTracks
                   && ((aPos >= ntrkPos) || (aPos < ttrkPos)) )
                  changeTrack(aPos);

               std::size_t const newPS = (bd.second * tbNum) / tbDen;

               if ( newPS != oldPosS )
                {
                  oldPosS = newPS;

                  auto const h { newPS / 3600U };
                  auto const m { (newPS % 3600U) / 60U };
                  auto const s { newPS % 60U };

                  (std::cout << "pos: " << std::setfill('0')
                     << std::setw(2) << h << ':'
                     << std::setw(2) << m << ':'
                     << std::setw(2) << s << " / "
                     << std::setw(2) << lenH << ':'
                     << std::setw(2) << lenM << ':'
                     << std::setw(2) << lenS << " ("
                     << (newPS * 100) / lenF << "%)  \r").flush();
                }
             }
          }
       }
      while ( loop && uLen );

      if ( toQuit )
         std::fill(cpStream, cpStream+len, std::uint8_t{0U});
      else if ( uLen )
         std::fill(stream, stream+uLen, std::uint8_t{0U});
    }

   ~audioData ()
    {
      SDL_Quit();

      av_free(swrBuff);
      swr_free(&swrCtx);
      avcodec_free_context(&codecCtx);
      avformat_close_input(&frmCtx);
    }
 };

std::unordered_map<AVSampleFormat, int> const audioData::formM
 { { AV_SAMPLE_FMT_U8,  AUDIO_U8 },
   { AV_SAMPLE_FMT_S16, AUDIO_S16SYS },
   { AV_SAMPLE_FMT_S32, AUDIO_S32SYS },
   { AV_SAMPLE_FMT_FLT, AUDIO_F32SYS } }; 


void sdl_audio_callback (void * userdata, std::uint8_t * stream, int len)
 { static_cast<audioData *>(userdata)->sdl_acb(stream, len); }


enum struct seek { noSeek, seekB1, seekB2, seekB3, seekF1, seekF2,
                   seekF3, seekNT, seekPT };

class keybHandl
 {
   public: 
      enum struct key { klP, klQ, kLeft, kRight, kUp, kDown, kPageUp,
                        kPageDown, kHome, kEnd, kUndef };

   private:

      termios oldT;

      std::unordered_map<std::string, key>  km;

      void addSpecialKey (char const id[3U], key const & localKey)
       {
         auto const pStr { tgetstr(id, nullptr) };

         if ( nullptr == pStr )
            std::cerr << "error loading keyboard key [" << id << "]"
               << std::endl;
         else
            km.emplace(pStr, localKey);
       }

   public:

      keybHandl ()
       {
         using namespace std::string_literals;

         if ( tcgetattr(STDIN_FILENO, &oldT) )
            throw std::runtime_error("error from tcgetattr() ["s
                                     +std::strerror(errno)+"]");
       }

      ~keybHandl ()
       { tcsetattr(STDIN_FILENO, TCSANOW, &oldT); }

      void initialize (bool withTracks)
       {
         using namespace std::string_literals;

         termios newT { oldT };

         newT.c_lflag &= ~ICANON;
         newT.c_lflag &= ~ECHO;

         newT.c_cc[VMIN]  = 0;
         newT.c_cc[VTIME] = 10;

         if ( tcsetattr(STDIN_FILENO, TCSANOW, &newT) )
            throw std::runtime_error("error from tcsetattr() ["s
                                     +std::strerror(errno)+"]");

         km.emplace("p", key::klP);  // lower p (play/pause)
         km.emplace("q", key::klQ);  // lower q (quit)

         auto const termName { std::getenv("TERM") };

         if ( nullptr == termName )
            std::cerr << "no terminal name detected; arrows keys disabled"
               << std::endl;
         else if ( 0 >= tgetent(nullptr, termName) )
            std::cerr << "error loading terminal; arrows keys disabled"
               << std::endl;
         else
          {
            auto const pStr { tgetstr("ks", nullptr) };

            // send ksmx sequence (if available) to enable correct
            // (as defined in terminal description) arrow keys codes
            if ( nullptr == pStr )
               std::cerr << "error loading ksmx code; the arrows keys can"
                  " unavailable" << std::endl;
            else
               (std::cout << pStr).flush();

            addSpecialKey("kl", key::kLeft);     // left arrow
            addSpecialKey("kr", key::kRight);    // right arrow
            addSpecialKey("ku", key::kUp);       // up arrow
            addSpecialKey("kd", key::kDown);     // up arrow
            addSpecialKey("kP", key::kPageUp);   // page up
            addSpecialKey("kN", key::kPageDown); // page down

            if ( withTracks )
             {
               addSpecialKey("kh", key::kHome);     // home
               addSpecialKey("@7", key::kEnd);      // end
             }
          }
       }

      auto const & getKm () const 
       { return km; } 
 };


class audioPlayer
 {
   private:

      std::thread              keybT;  // thread for keyboard reading
      std::mutex               pM;     // pause mutex
      std::condition_variable  pCV;    // pause condition variable
      std::atomic<bool>        aLoop  { true };
      std::atomic<seek>        aSeek  { seek::noSeek };

      bool pause { false };

      audioData  ad;
      keybHandl  kh;

      int decodeAV (AVCodecContext & avCtx, AVFrame & avFrm,
                    int & gotFrame, AVPacket const & avPkt)
       {
         gotFrame = 0;

         int ret { avcodec_send_packet(&avCtx, &avPkt) };

         if ( 0 > ret )
            ret = (ret == AVERROR_EOF ? 0 : ret);
         else if ( 0 <= (ret = avcodec_receive_frame(&avCtx, &avFrm)) )
            gotFrame = 1;
         else if ( (AVERROR(EAGAIN) == ret) || (AVERROR_EOF == ret) )
            ret = 0;

         return ret;
       }

      void setPause (bool newVal)
       {
         auto pul { std::unique_lock<std::mutex>(pM) };

         pause = newVal;

         pul.unlock();

         pCV.notify_one();

         SDL_PauseAudioDevice(ad.devId, pause);
       }

      void keybReader ()
       {
         using key = keybHandl::key;

         std::array<char, 16U>  buff;

         while ( aLoop )
          {
            using namespace std::string_literals;

            auto const val { read(STDIN_FILENO, buff.data(), buff.size()) };

            if ( 0 > val )
               throw std::runtime_error("error reading the keyboard ["s
                                        +std::strerror(errno)+"]");
            else if ( 0 < val )
             {
               buff[val] = '\0';

               auto const kmIt { kh.getKm().find(buff.data()) };
               auto const kVal { kmIt == kh.getKm().cend()
                                    ? key::kUndef : kmIt->second };

               switch ( kVal )
                {
                  case key::klP:
                     setPause( ! pause ); break;
                  case key::klQ:
                     aLoop=false; ad.toQuit=true; setPause(false); break;
                  case key::kLeft:
                     aSeek=seek::seekB1; setPause(false); break;
                  case key::kRight:
                     aSeek=seek::seekF1; setPause(false); break;
                  case key::kUp:
                     aSeek=seek::seekF2; setPause(false); break;
                  case key::kDown:
                     aSeek=seek::seekB2; setPause(false); break;
                  case key::kPageUp:
                     aSeek=seek::seekF3; setPause(false); break;
                  case key::kPageDown:
                     aSeek=seek::seekB3; setPause(false); break;
                  case key::kHome:
                     aSeek=seek::seekPT; setPause(false); break;
                  case key::kEnd:
                     aSeek=seek::seekNT; setPause(false); break;
                  default:
                     break;
                }
             }
          }
       }

      void seekStream (seek const & seekVal)
       {
         std::int64_t  newPos      { 0 };
         int           forwardFlag { 0 };

         switch ( seekVal )
          {
            case seek::seekB1: newPos =        -10; forwardFlag = 1; break;
            case seek::seekF1: newPos =        +10; forwardFlag = 0; break;
            case seek::seekB2: newPos =        -60; forwardFlag = 1; break;
            case seek::seekF2: newPos =        +60; forwardFlag = 0; break;
            case seek::seekB3: newPos =       -600; forwardFlag = 1; break;
            case seek::seekF3: newPos =       +600; forwardFlag = 0; break;
            case seek::seekPT: newPos = ad.ptrkPos; forwardFlag = 2; break;
            case seek::seekNT: newPos = ad.ntrkPos; forwardFlag = 2; break;

            default: forwardFlag = -1; break;
          }

         if ( -1 != forwardFlag )
          {
            if ( 2 == forwardFlag )
               forwardFlag = (newPos < ad.aPos);
            else
             {
               newPos *= ad.tbDen;
               newPos /= ad.tbNum;
               newPos += ad.aPos;
             }

            SDL_PauseAudioDevice(ad.devId, 1);

            ad.cb.purge();

            ad.aPurged = true;
         
            av_seek_frame(ad.frmCtx, ad.aStreamI, newPos,
                          forwardFlag ? AVSEEK_FLAG_BACKWARD : 0);

            SDL_PauseAudioDevice(ad.devId, 0);
          }
       }

   public: 

      audioPlayer (std::string const & fileName)
       {
         ad.initialize(fileName);
         kh.initialize(ad.numTracks > 0U);

         keybT = std::thread(&audioPlayer::keybReader, this);

         if ( 0 > av_seek_frame(ad.frmCtx, ad.aStreamI, 0, 0) )
            throw std::runtime_error("av_seek_frame error");

         AVPacket   aPck;
         AVFrame *  aFrm { av_frame_alloc() };

         int        fc { 0 };

         if ( nullptr == aFrm )
            throw std::runtime_error("error allocating audio frame");

         SDL_PauseAudioDevice(ad.devId, pause);

         while ( aLoop )
          {
            // pause mutex safe area
             {
               auto pl { std::unique_lock<std::mutex>(pM) };
               pCV.wait(pl, [&]{ return pause == false; });
             }

            auto const seekVal { aSeek.exchange(seek::noSeek) };

            if ( seekVal != seek::noSeek )
               seekStream(seekVal);

            if ( 0 <= av_read_frame(ad.frmCtx, &aPck) )
             {
               if ( aPck.stream_index == ad.aStreamI )
                {
                  if ( 0 > decodeAV(*ad.codecCtx, *aFrm, fc, aPck) )
                     std::cerr << "error from decodeAV()" << std::endl;

                  // if frame completed
                  if ( fc )
                   {
                     // if format conversion required
                     if ( true == ad.frmConv )
                      {
                        int  inSmpl { aFrm->nb_samples };
                        int  outSmpl;

                        do
                         {
                           outSmpl = swr_convert(ad.swrCtx, &ad.swrBuff,
                                        ad.maxOutFrames,
                                        (std::uint8_t const **) aFrm->data,
                                        inSmpl);

                           if ( 0 < outSmpl )
                            {
                              auto const size
                               { av_samples_get_buffer_size
                                 (nullptr, ad.codecCtx->channels,
                                  outSmpl, ad.defFfmpegFormat, 1) };

                              std::vector<std::uint8_t> vc(size);

                              std::copy(ad.swrBuff, ad.swrBuff+size,
                                        vc.data());

                              ad.cb.push(std::move(vc), aPck.pts);
                            }
                           else if ( 0 > outSmpl )
                              std::cerr << "error " << outSmpl
                                 << " swr converting" << std::endl;

                           inSmpl = 0;
                         }
                        while ( outSmpl == ad.maxOutFrames );
                      }
                     else
                      {
                        auto const size
                         { av_samples_get_buffer_size
                           (nullptr, ad.codecCtx->channels,
                            aFrm->nb_samples, ad.codecCtx->sample_fmt, 1) };

                        std::vector<std::uint8_t> vc(size);

                        std::copy(*(aFrm->data), *(aFrm->data)+size,
                                  vc.data());

                        ad.cb.push(std::move(vc), aPck.pts);
                      }
                   }
                }

               av_packet_unref(&aPck);
             }
            else
               aLoop = false;
          }
       }

      ~audioPlayer ()
       { keybT.join(); }
 };

int main (int argc, char * argv[])
 {
   if ( 2 == argc )
      audioPlayer{argv[1]};
   else
      std::cerr << "usage: " << argv[0] << " <media file>" << std::endl;

   return EXIT_SUCCESS;
 }


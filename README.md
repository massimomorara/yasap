# Yasap
## Yet Another Simple Audio Player

Yasap is a simple command line audio player, that uses FFmpeg and SDL2 libraries, with support for multiple tracks in a single audio file trough Cuesheets embedded in a "cuesheet" named metadata tag (Vorbis comment, Matroska tag, MP3 tag, etc).

Written in C++14, in a Linux Debian stable (stretch) platform, should be easily portable in others fairly recently Linux/Unix platforms.

The code is in a unique C++14 source file (`yasap.cpp`) and a simple `Makefile` show how to compile it and which libraries to link.

The `Makefile` uses `g++` but the code compile also with `clang++`.

The use is simple: the `yasap` executable accept only one command line parameter, the audio file, and play it.

By example

```
$ ./yasap song.flac
```

When playing, the following keys/commands are enabled

```
  p            switch pause / play
  q            terminate and exit
  right arrow  seek forward 10 seconds
  left arrow   seek backward 10 seconds
  up arrow     seek forward 1 minute
  down arrow   seek backward 1 minute
  page up      seek forward 10 minutes
  page down    seek backward 10 minutes 
  home         seek to preceding track (only for files with multiple tracks)
  end          seek to following track (only for files with multiple tracks)
```


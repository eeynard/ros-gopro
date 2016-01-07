#!/usr/bin/env bash

apt-get -y install yasm

# FFMPEG
apt-get -y install build-essential checkinstall git libfaac-dev libgpac-dev \
    libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev librtmp-dev libtheora-dev \
    libvorbis-dev pkg-config texi2html zlib1g-dev \
    libass-dev libsdl1.2-dev \
    libav-tools ubuntu-restricted-extras libavcodec-extra-54 \
    libavformat-extra-54 libdc1394-22-dev libdc1394-22 libdc1394-utils

# Encoders
apt-get -y install libx264-dev libmp3lame-dev libopus-dev

mkdir ~/ffmpeg_sources

cd ~/ffmpeg_sources

wget http://ffmpeg.org/releases/ffmpeg-snapshot.tar.bz2
tar xjvf ffmpeg-snapshot.tar.bz2

cd ffmpeg

PATH="$HOME/bin:$PATH" PKG_CONFIG_PATH="$HOME/ffmpeg_build/lib/pkgconfig" ./configure \
  --prefix="$HOME/ffmpeg_build" \
  --pkg-config-flags="--static" \
  --extra-cflags="-I$HOME/ffmpeg_build/include" \
  --extra-ldflags="-L$HOME/ffmpeg_build/lib" \
  --bindir="$HOME/bin" \
  --enable-ffplay \
  --enable-gpl \
  --enable-libass \
  --enable-libfreetype \
  --enable-libmp3lame \
  --enable-libopus \
  --enable-libtheora \
  --enable-libvorbis \
  --enable-libx264 \
  --enable-nonfree

PATH="$HOME/bin:$PATH" make
make install
make distclean
hash -r

cp ~/bin/ff* /usr/local/bin
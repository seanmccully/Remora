#!/usr/bin/env sh
# Build and Install: https://github.com/WiringPi/WiringPi
# Install with prefix /usr instead of /usr/local - otherwise you will need to -I/usr/local/include to the compile command. 
# I did this with editing /usr/share/linuxcnc/Makefile.modinc and adding "-I/usr/local/include" to RTFLAGS.
# I am able to loadrt remora with halcmd, but have not gotten a linuxcnc actually working with this yet. 
halcomile=$(command -v halcompile)
EXTRA_LDFLAGS="-lwiringPi" sudo $halcompile  --install remora.c


INSIGHT 3D NG, VERSION 0.4.0
============================

COMPILING ON MACOS
------------------

Compiling on macOS requires [Homebrew](https://brew.sh)

```terminal
brew install opencv@2 freetype sdl libxml2 lapack gtk+

# the following three dependencies are not available in homebrew-core:
brew tap mikkelee/mikkelee
brew install ann opensift sba

mkdir build; cd build
cmake ..
make
```

COMPILING ON LINUX
------------------

To compile insight3dng under Linux, you must have the following libraries: 

	- opencv
	- opengl
	- SDL
	- libxml2 (to parse xml files)
	- lapack 
	- blas (min. 1.2-8 (Debian/Ubuntu))
	- libgtk+-2.0

Compilation is done via `cmake`.

COMPILING ON WINDOWS
--------------------

You'll need the same libraries. MSVC files can be generated via `cmake`.

# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.21

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2021.3.2\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2021.3.2\bin\cmake\win\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/DroneClient.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/DroneClient.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/DroneClient.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/DroneClient.dir/flags.make

CMakeFiles/DroneClient.dir/src/main.cpp.obj: CMakeFiles/DroneClient.dir/flags.make
CMakeFiles/DroneClient.dir/src/main.cpp.obj: CMakeFiles/DroneClient.dir/includes_CXX.rsp
CMakeFiles/DroneClient.dir/src/main.cpp.obj: ../src/main.cpp
CMakeFiles/DroneClient.dir/src/main.cpp.obj: CMakeFiles/DroneClient.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/DroneClient.dir/src/main.cpp.obj"
	C:\PROGRA~1\JETBRA~1\CLION2~1.2\bin\mingw\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DroneClient.dir/src/main.cpp.obj -MF CMakeFiles\DroneClient.dir\src\main.cpp.obj.d -o CMakeFiles\DroneClient.dir\src\main.cpp.obj -c C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\src\main.cpp

CMakeFiles/DroneClient.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DroneClient.dir/src/main.cpp.i"
	C:\PROGRA~1\JETBRA~1\CLION2~1.2\bin\mingw\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\src\main.cpp > CMakeFiles\DroneClient.dir\src\main.cpp.i

CMakeFiles/DroneClient.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DroneClient.dir/src/main.cpp.s"
	C:\PROGRA~1\JETBRA~1\CLION2~1.2\bin\mingw\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\src\main.cpp -o CMakeFiles\DroneClient.dir\src\main.cpp.s

CMakeFiles/DroneClient.dir/src/UWB.cpp.obj: CMakeFiles/DroneClient.dir/flags.make
CMakeFiles/DroneClient.dir/src/UWB.cpp.obj: CMakeFiles/DroneClient.dir/includes_CXX.rsp
CMakeFiles/DroneClient.dir/src/UWB.cpp.obj: ../src/UWB.cpp
CMakeFiles/DroneClient.dir/src/UWB.cpp.obj: CMakeFiles/DroneClient.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/DroneClient.dir/src/UWB.cpp.obj"
	C:\PROGRA~1\JETBRA~1\CLION2~1.2\bin\mingw\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DroneClient.dir/src/UWB.cpp.obj -MF CMakeFiles\DroneClient.dir\src\UWB.cpp.obj.d -o CMakeFiles\DroneClient.dir\src\UWB.cpp.obj -c C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\src\UWB.cpp

CMakeFiles/DroneClient.dir/src/UWB.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DroneClient.dir/src/UWB.cpp.i"
	C:\PROGRA~1\JETBRA~1\CLION2~1.2\bin\mingw\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\src\UWB.cpp > CMakeFiles\DroneClient.dir\src\UWB.cpp.i

CMakeFiles/DroneClient.dir/src/UWB.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DroneClient.dir/src/UWB.cpp.s"
	C:\PROGRA~1\JETBRA~1\CLION2~1.2\bin\mingw\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\src\UWB.cpp -o CMakeFiles\DroneClient.dir\src\UWB.cpp.s

CMakeFiles/DroneClient.dir/src/TCP_Socket.cpp.obj: CMakeFiles/DroneClient.dir/flags.make
CMakeFiles/DroneClient.dir/src/TCP_Socket.cpp.obj: CMakeFiles/DroneClient.dir/includes_CXX.rsp
CMakeFiles/DroneClient.dir/src/TCP_Socket.cpp.obj: ../src/TCP_Socket.cpp
CMakeFiles/DroneClient.dir/src/TCP_Socket.cpp.obj: CMakeFiles/DroneClient.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/DroneClient.dir/src/TCP_Socket.cpp.obj"
	C:\PROGRA~1\JETBRA~1\CLION2~1.2\bin\mingw\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DroneClient.dir/src/TCP_Socket.cpp.obj -MF CMakeFiles\DroneClient.dir\src\TCP_Socket.cpp.obj.d -o CMakeFiles\DroneClient.dir\src\TCP_Socket.cpp.obj -c C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\src\TCP_Socket.cpp

CMakeFiles/DroneClient.dir/src/TCP_Socket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DroneClient.dir/src/TCP_Socket.cpp.i"
	C:\PROGRA~1\JETBRA~1\CLION2~1.2\bin\mingw\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\src\TCP_Socket.cpp > CMakeFiles\DroneClient.dir\src\TCP_Socket.cpp.i

CMakeFiles/DroneClient.dir/src/TCP_Socket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DroneClient.dir/src/TCP_Socket.cpp.s"
	C:\PROGRA~1\JETBRA~1\CLION2~1.2\bin\mingw\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\src\TCP_Socket.cpp -o CMakeFiles\DroneClient.dir\src\TCP_Socket.cpp.s

CMakeFiles/DroneClient.dir/src/Mavlink.cpp.obj: CMakeFiles/DroneClient.dir/flags.make
CMakeFiles/DroneClient.dir/src/Mavlink.cpp.obj: CMakeFiles/DroneClient.dir/includes_CXX.rsp
CMakeFiles/DroneClient.dir/src/Mavlink.cpp.obj: ../src/Mavlink.cpp
CMakeFiles/DroneClient.dir/src/Mavlink.cpp.obj: CMakeFiles/DroneClient.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/DroneClient.dir/src/Mavlink.cpp.obj"
	C:\PROGRA~1\JETBRA~1\CLION2~1.2\bin\mingw\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DroneClient.dir/src/Mavlink.cpp.obj -MF CMakeFiles\DroneClient.dir\src\Mavlink.cpp.obj.d -o CMakeFiles\DroneClient.dir\src\Mavlink.cpp.obj -c C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\src\Mavlink.cpp

CMakeFiles/DroneClient.dir/src/Mavlink.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DroneClient.dir/src/Mavlink.cpp.i"
	C:\PROGRA~1\JETBRA~1\CLION2~1.2\bin\mingw\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\src\Mavlink.cpp > CMakeFiles\DroneClient.dir\src\Mavlink.cpp.i

CMakeFiles/DroneClient.dir/src/Mavlink.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DroneClient.dir/src/Mavlink.cpp.s"
	C:\PROGRA~1\JETBRA~1\CLION2~1.2\bin\mingw\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\src\Mavlink.cpp -o CMakeFiles\DroneClient.dir\src\Mavlink.cpp.s

# Object files for target DroneClient
DroneClient_OBJECTS = \
"CMakeFiles/DroneClient.dir/src/main.cpp.obj" \
"CMakeFiles/DroneClient.dir/src/UWB.cpp.obj" \
"CMakeFiles/DroneClient.dir/src/TCP_Socket.cpp.obj" \
"CMakeFiles/DroneClient.dir/src/Mavlink.cpp.obj"

# External object files for target DroneClient
DroneClient_EXTERNAL_OBJECTS =

DroneClient.exe: CMakeFiles/DroneClient.dir/src/main.cpp.obj
DroneClient.exe: CMakeFiles/DroneClient.dir/src/UWB.cpp.obj
DroneClient.exe: CMakeFiles/DroneClient.dir/src/TCP_Socket.cpp.obj
DroneClient.exe: CMakeFiles/DroneClient.dir/src/Mavlink.cpp.obj
DroneClient.exe: CMakeFiles/DroneClient.dir/build.make
DroneClient.exe: CMakeFiles/DroneClient.dir/linklibs.rsp
DroneClient.exe: CMakeFiles/DroneClient.dir/objects1.rsp
DroneClient.exe: CMakeFiles/DroneClient.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable DroneClient.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\DroneClient.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/DroneClient.dir/build: DroneClient.exe
.PHONY : CMakeFiles/DroneClient.dir/build

CMakeFiles/DroneClient.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\DroneClient.dir\cmake_clean.cmake
.PHONY : CMakeFiles/DroneClient.dir/clean

CMakeFiles/DroneClient.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\cmake-build-debug C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\cmake-build-debug C:\Users\ColinLaganier\Documents\Imperial\Thesis\AutonomousDrone\DroneClient\cmake-build-debug\CMakeFiles\DroneClient.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/DroneClient.dir/depend


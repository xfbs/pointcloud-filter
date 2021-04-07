# Pointcloud Filter

This is a multithreaded C++ script that takes in pointcloud data (which are
represented as binary files containing 4-tuples of `(f32, f32, f32, f32)` representing
the X, Y, Z coordinates as well as an intensity.

It takes these points, computes their angle, and filters points down to those within a
specific range of angles, configured in the script. This is done to allow the dataset
to better match the data that can be expected from a real-world sensor that might
not have as big a range.

## Building

You need a recent operating system (Ubuntu), CMake and a C++ compiler installed.
To install these using the APT package manager (Ubuntu, Debian), run

    apt install build-essential cmake

Create a build folder, initialize CMake and build the code.

    mkdir build
    cd build
    cmake ..
    make

## License

MIT.

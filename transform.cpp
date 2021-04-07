#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <optional>

using namespace std;
namespace fs = std::filesystem;

#define unused(x) (void)x

struct directory_pair {
    fs::path source;
    fs::path target;
};

struct point {
    float x;
    float y;
    float z;
    float i;
};

const directory_pair directory_pairs[2] = {
    {"testing/velodyne", "testing/velodyne_filtered2"},
    {"training/velodyne", "training/velodyne_filtered2"},
};

std::vector<point> filter_points(vector<point> &input) {
    vector<point> output = input;

    return output;
}

int main(int argc, char *argv[]) {
    unused(argc);
    unused(argv);

    for(auto &pair: directory_pairs) {
        fs::create_directory(pair.target);

        // get list of paths
        vector<fs::path> source_files;
        for(auto &p: fs::directory_iterator(pair.source)) {
            source_files.push_back(p.path());
        }

        for(auto &path: source_files) {
            // print current file
            cout << path << endl;

            // read source points in completely
            vector<point> source_data;
            ifstream source_file(path, ios::binary);
            point data;
            while(source_file.read(reinterpret_cast<char *>(&data), sizeof(data))) {
                source_data.push_back(data);
            }

            // filter points
            vector<point> target_data = filter_points(source_data);

            // write points out
            fs::path target_path = pair.target / path.filename();
            ofstream target_file(target_path, ios::binary);
            for(auto &p: target_data) {
                if(!target_file.write(reinterpret_cast<char *>(&p), sizeof(p))) {
                    cerr << "Error writing out file!" << endl;
                    return 1;
                }
            }
        }
    }
}

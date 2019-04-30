#ifndef SFILE_HPP
#define SFILE_HPP

#include <stdio.h>
#include <stdlib.h>
#include <zconf.h>
#include <string>
#include <dirent.h>

namespace shawn {
    class SFile {
    public:
        bool deleteDir(std::string dir) {
            DIR *dirp;
            struct dirent *dp;
            struct stat st;
            if (0 != access(dir.c_str(), F_OK) ) {
                return false;
            }
            if (0 > stat(dir.c_str(), &st)) {
                return false;
            }
            if (S_ISREG(st.st_mode)) {
                remove(dir.c_str());
            }
            if (S_ISDIR(st.st_mode)) {
                dirp = opendir(dir.c_str());
                while ( (dp=readdir(dirp)) != NULL ) {
                    if ((0 == strcmp(".", dp->d_name)) || (0 == strcmp("..", dp->d_name))) {
                        continue;
                    }
                    std::string sub_dir = dir + '/' + dp->d_name;
                    deleteDir(sub_dir);
                }
                closedir(dirp);
                rmdir(dir.c_str());
            }
        }
    };
}

#endif //SFILE_HPP
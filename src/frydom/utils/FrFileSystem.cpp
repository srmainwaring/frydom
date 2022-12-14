// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#include <iostream>

#include "FrFileSystem.h"
#include "cppfs/fs.h"

//#include "platform_folders.h"

#ifndef _WIN32

#include <unistd.h>
#include <pwd.h>
#include <limits.h>

#else

#include <windows.h>
#include <Lmcons.h>
#include <direct.h>

#endif

#include <vector>
#include <cppfs/FilePath.h>


namespace frydom {

  std::string FrFileSystem::cwd() {

    char currentPath[FILENAME_MAX];
    std::string path_str;

    #ifndef _WIN32
    if (getcwd(currentPath, FILENAME_MAX) != nullptr) {
      #else
      if (_getcwd(currentPath, FILENAME_MAX) != nullptr) {
      #endif
      path_str = std::string(currentPath);
    }
    return path_str;

  }

  std::string FrFileSystem::get_home() {  // FIXME : non portable ...
    std::string res;
    int uid = getuid();
    const char *homeEnv = std::getenv("HOME");
    if (uid != 0 && homeEnv) {
      //We only acknowlegde HOME if not root.
      res = homeEnv;
      return res;
    }
    struct passwd *pw = getpwuid(uid);
    if (!pw) {
      throw std::runtime_error("Unable to get passwd struct.");
    }
    const char *tempRes = pw->pw_dir;
    if (!tempRes) {
      throw std::runtime_error("User has no home directory");
    }
    res = tempRes;
    return res;
  }

//  std::string FrFileSystem::abspath(const std::string &path) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }
//
//  std::string FrFileSystem::normpath(const std::string& path) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }

  bool FrFileSystem::mkdir(const std::string &path) {
    cppfs::FileHandle fh = cppfs::fs::open(path);
    return mkdir(fh);
  }

  bool FrFileSystem::mkdir(cppfs::FileHandle &fh) {

    if (fh.exists()) return true;

    cppfs::FileHandle parent_dir = fh.parentDirectory();
    if (mkdir(parent_dir)) {
      if (!fh.createDirectory()) {
        std::cerr << "Cannot create directory \"" << fh.path() << "\". Permission denied." << std::endl;
        exit(EXIT_FAILURE);
      }
      return true;
    }
    return false;
  }

  bool FrFileSystem::exists(const std::string &path) {
    cppfs::FileHandle fh = cppfs::fs::open(path);
    return fh.exists();
  }

  bool FrFileSystem::isfile(const std::string &path) {
    cppfs::FileHandle fh = cppfs::fs::open(path);
    return fh.isFile();
  }

  bool FrFileSystem::isdir(const std::string &path) {
    cppfs::FileHandle fh = cppfs::fs::open(path);
    return fh.isDirectory();
  }

//  std::string FrFileSystem::basename(const std::string &path) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }
//
//  std::string FrFileSystem::dirname(const std::string &path) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }
//
  bool FrFileSystem::isabs(const std::string &path) {
    cppfs::FilePath fp(path);
    return fp.isAbsolute();
  }

  std::string FrFileSystem::join(std::vector<std::string> paths) {
    cppfs::FilePath fp;
    for (const auto &path: paths) {
      fp = fp.resolve(path);
    }
    return fp.resolved();
  }

//  std::pair<std::string, std::string> FrFileSystem::split(const std::string &path) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }
//
//  std::pair<std::string, std::string> FrFileSystem::splitdrive(const std::string &path) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }
//
//  std::pair<std::string, std::string> FrFileSystem::splitext(const std::string &path) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }
//
//  void FrFileSystem::copy(const std::string &src, const std::string &dst) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }
//
//  void FrFileSystem::copytree(const std::string &src, const std::string &dst) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }
//
//  void FrFileSystem::rmtree(const std::string &path) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }
//
//  void FrFileSystem::move(const std::string &src, const std::string &dst) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }

//  void FrFileSystem::make_archive_tgz(const std::string &archive_name, const std::string &root_dir) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }
//
//  void FrFileSystem::make_archive_zip(const std::string &archive_name, const std::string &root_dir) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }

  std::string FrFileSystem::get_login() {

//    #ifndef _WIN32
//    char username[LOGIN_NAME_MAX];
//    getlogin_r(username, LOGIN_NAME_MAX);
//    return username;
//    #else
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//    #endif



    #ifndef _WIN32

    struct passwd *pwd = getpwuid(getuid());
    if (pwd) {
      return pwd->pw_name;
    } else {
      return "(?)";
    }

    #else
    std::cerr << "Not implemented yet..." << std::endl;
    exit(EXIT_FAILURE);
    #endif

  }

  std::string FrFileSystem::get_hostname() {

    #ifndef _WIN32
    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);
    return hostname;
    #else
    std::cerr << "Not implemented yet..." << std::endl;
    exit(EXIT_FAILURE);
    #endif

  }

//  void FrFileSystem::rename(const std::string &src, const std::string &dst) {
//    std::cerr << "Not implemented yet..." << std::endl;
//    exit(EXIT_FAILURE);
//  }


}  // end namespace frydom

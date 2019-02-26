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

#ifndef FRYDOM_FRLOGMANAGER_H
#define FRYDOM_FRLOGMANAGER_H

#include <cppfs/fs.h>
#include <cppfs/FileHandle.h>
#include <cppfs/FilePath.h>

#include "frydom/core/common/FrConvention.h"

namespace frydom {

    //Forward declaration
    class FrBody_;
    class FrOffshoreSystem_;

    class FrLogManager {
    private:

        FRAME_CONVENTION m_logFrameConvention;

        cppfs::FilePath m_outputPath;

    public:

        /// Constructor for a log manager service
        /// \param system system providing the log manager service
        explicit FrLogManager();


        /// Set the frame convention for the logs
        /// \param fc frame convention (NED/NWU)
        void SetLogFrameConvention(FRAME_CONVENTION fc);

        /// Get the frame convention for the logs
        /// \return frame convention (NED/NWU)
        FRAME_CONVENTION GetLogFrameConvention() const;

        /// Set the path for the output directory, containing all log files
        /// \param path path for the output directory
        void SetLogOutputPath(std::string path);

        /// Get the path for the output directory, containing all log files
        /// \return path for the output directory
        std::string GetLogOutputPath() const;


        ///Initialize the log manager serice
        void Initialize();

        /// Declare a new system log instance : create the run and system log directories
        /// and return the path to the system log file
        /// \param system system for which a log is declared
        /// \return path to the system log file
        std::string NewSystemLog(FrOffshoreSystem_* system);

        /// Declare a new body log instance : create the body log directory and return the path to the body log file
        /// \param body body for which a log is declared
        /// \return path to the body log file
        std::string NewBodyLog(FrBody_* body);

    private:
        /// Read the config file
        void ReadConfig();
    };

} // end namespace frydom

#endif //FRYDOM_FRLOGMANAGER_H

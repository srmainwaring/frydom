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


#ifndef FRYDOM_FROBJECT_H
#define FRYDOM_FROBJECT_H

#include "boost/lexical_cast.hpp"
#include "boost/uuid/uuid_io.hpp"
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_generators.hpp"

namespace frydom {

  // Forward declarations
  class FrOffshoreSystem;
//    class FrPathManager;


  /**
   * \class FrObject
   * \brief Class for defining objects in FRyDoM.
   */
  class FrObject {

    // TODO : abandonner les uuid boost au profit d'un nameServer qui s'assure de l'unicite des noms donnes par l'utilisateur
    // La resolution de nom devra avoir lieu lors de l'initialisation des classes et le nameServer fera parti du OffshoreSystem...

   private:
    std::string m_UUID;         ///< Universal Unique Identifier, generated by boost
//    std::string m_name;         ///< name of the object

   protected:

    // Logging
//        bool m_isLogged = false;                        ///< Is the object logged?
//
//        std::unique_ptr<hermes::Message> m_message;     ///< Hermes message, containing the fields to be logged
//
//        std::shared_ptr<FrPathManager> m_pathManager;   ///< pointer to the path manager, in charge of building the path
//                                                        ///< to the log file of this object

   public:

    /// Default constructor
    FrObject();

    // ----- UUID manipulation -------------------------------------------------------------------------------------

    /// Get the universal unique identifier
    /// \return universal unique identifier
    std::string GetUUID() const;

    /// Get a 5 digits shortcut of the universal unique identifier
    /// \return 5 digits shortcut of the universal unique identifier
    std::string GetShortenUUID() const;


    // ----- Name settings -----------------------------------------------------------------------------------------

//    /// Gets the name of the object as C Ascii null-terminated string -for reading only!
//    const char *GetName() const;
//
//    /// Sets the name of this object, as ascii string
//    void SetName(const char myname[]);
//
//    /// Gets the name of the object as C Ascii null-terminated string.
//    std::string GetNameString() const;
//
//    /// Sets the name of this object, as std::string
//    void SetNameString(const std::string &myname);


    // ----- Logging methods ---------------------------------------------------------------------------------------

//        /// Check if the object is logged
//        /// \return true if the object is logged
//        bool IsLogged();
//
//        /// Set the object to be logged or not
//        /// \param isLogged true if the object is to be logged
//        void SetLogged(bool isLogged);
//
//        /// Initialize the logging of the object : build the path, create the directory, add the fields to be logged, etc.
//        /// \param path path of the parent object, to build the path of the present object
//        void InitializeLog(const std::string& path);
//
//        /// Initialize the logging of the dependencies (attributes of the present object)
//        /// \param path path of the present object, to give to the InitializeLog of the dependencies to build their log path
//        virtual void InitializeLog_Dependencies(const std::string& path) {};
//
//        /// Set the pointer to the path manager service, in charge of building the path of every object to be logged
//        /// \param manager shared pointer to the path manager service
//        void SetPathManager(const std::shared_ptr<FrPathManager>& manager);
//
//        /// Get the shared pointer to the path manager service
//        /// \return shared pointer to the path manager service
//        std::shared_ptr<FrPathManager> GetPathManager() const;
//
//        /// Get the frame convention used in the logging
//        /// \return Frame convention used in logging (NED/NWU)
//        FRAME_CONVENTION GetLogFrameConvention() const;
//
//        /// Clear the Hermes message, from all fields and serializer
//        void ClearMessage();
//
//   protected:
//
//        /// Serialize and send the message
//        void SendLog();
//
//        /// Build the path to the log file, create the directory and add a csv serializer
//        /// \param rootPath path of the parent directory
//        /// \return path of the present log directory
//        virtual std::string BuildPath(const std::string& rootPath);
//
//        /// Add the fields to the Hermes message
//        virtual void AddFields() {};


    // ----- Virtual methods ---------------------------------------------------------------------------------------

   public:


    /// Base method for Initialization of FryDoM objects
    ///
    /// This must be overrided in children classes in case of a need for special initialization at the beginning
    /// of a computation. Every Initialize() methods must be called indirectly when the call to
    /// FrOffshoreSystem::Initialize() is done.
    virtual void Initialize() = 0;

    /// This function is called at the end of the time step, after the last step of the integration scheme.
    virtual void StepFinalize();

  };

}  // end namespace frydom

#endif //FRYDOM_FROBJECT_H

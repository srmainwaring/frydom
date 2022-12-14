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

#ifndef FRYDOM_FRNODE_H
#define FRYDOM_FRNODE_H


#include "chrono/physics/ChMarker.h"
#include "FrRotation.h"
#include "FrFrame.h"
#include "FrObject.h"

#include "frydom/core/link/links_lib/FrLink.h"

#include "frydom/logging/FrLoggable.h"


namespace frydom {

  // Forward declarations
  class FrNode;

  namespace internal {

    struct FrMarker : public chrono::ChMarker {

      FrNode *m_frydomNode;

      explicit FrMarker(FrNode *node);

    };

    std::shared_ptr<FrMarker> GetChronoMarker(std::shared_ptr<FrNode> node);

  } // end namespace frydom::internal


  // Forward declarations
  class FrBody;

  class FrNodeAsset;

  /**
   * \class FrNode
   * \brief Class for defining nodes (in order to add links).
   */
  class FrNode : public FrObject, public FrLoggable<FrBody> {

   private:

    static const std::string s_type;

    std::shared_ptr<internal::FrMarker> m_chronoMarker;   ///< Chrono class for nodes/marker.

    // Asset for a node
    bool m_showAsset = false;
    std::shared_ptr<FrNodeAsset> m_asset;

   public:

    /// Default Constructor
    /// \param body body to which the node belongs
    FrNode(const std::string &name, FrBody *body);

    /// Destructor
//        ~FrNode() = default;

    /// Set if an asset is to be displayed
    /// \param showAsset true if a ForceAsset is to be displayed
    void ShowAsset(bool showAsset);;

    FrNodeAsset *GetAsset();

    /// Set node position and direction axis, with respect to body reference frame
    /// \param pos relative position of the frame node with respect to body reference frame
    /// \param e1 direction of the x-axis with respect to reference frame
    /// \param e2 direction of the y-axis with respect to reference frame
    /// \param e3 direction of the z-axis with respect to reference frame
    /// \param fc frame convention (NED/NWU)
    void Set(const Position &pos, const Direction &e1, const Direction &e2, const Direction &e3, FRAME_CONVENTION fc);


    /*
     * Setters
     */

    /// Set the node position with respect to body reference frame
    void SetPositionInBody(const Position &bodyPosition, FRAME_CONVENTION fc);

    void SetPositionInBody(const Position &refPos,
                           const double &heading,
                           const double &radial_distance,
                           const double &vertical_distance,
                           ANGLE_UNIT angle_unit,
                           FRAME_CONVENTION fc);

    void SetPositionInWorld(const Position &worldPosition, FRAME_CONVENTION fc);

    void SetPositionInWorld(const Position &refPos,
                            const double &heading,
                            const double &radial_distance,
                            const double &vertical_distance,
                            ANGLE_UNIT angle_unit,
                            FRAME_CONVENTION fc);

    void TranslateInBody(const Translation &translationInBody, FRAME_CONVENTION fc);

    void TranslateInBody(const Direction &directionBody, double distance, FRAME_CONVENTION fc);

    void TranslateInBody(double x, double y, double z, FRAME_CONVENTION fc);

    void TranslateInWorld(const Translation &translationInWorld, FRAME_CONVENTION fc);

    void TranslateInWorld(const Direction &directionWorld, double distance, FRAME_CONVENTION fc);

    void TranslateInWorld(double x, double y, double z, FRAME_CONVENTION fc);


    void SetOrientationInBody(const FrRotation &rotation);

    void SetOrientationInBody(const FrUnitQuaternion &quaternion);

    void RotateInBody(const FrRotation &rotation);

    void RotateInBody(const FrUnitQuaternion &quaternion);

    void RotateInWorld(const FrRotation &rotation);

    void RotateInWorld(const FrUnitQuaternion &quaternion);

    void RotateAroundXInBody(double angleRad, FRAME_CONVENTION fc);

    void RotateAroundYInBody(double angleRad, FRAME_CONVENTION fc);

    void RotateAroundZInBody(double angleRad, FRAME_CONVENTION fc);

    void RotateAroundXInWorld(double angleRad, FRAME_CONVENTION fc);

    void RotateAroundYInWorld(double angleRad, FRAME_CONVENTION fc);

    void RotateAroundZInWorld(double angleRad, FRAME_CONVENTION fc);


    /// Get the body pointer
    /// \return the body to which the node belongs
    inline FrBody *GetBody() const {
      return GetParent();
    }

    /// Get the node frame, given in the world reference frame
    /// \return the node frame in the world reference frame
    FrFrame GetFrameInWorld() const;

    /// Get the node frame, given in the body reference frame
    /// \return the node frame in the body reference frame
    FrFrame GetFrameInBody() const;

    /// Get the node frame with respect to the COG in body reference coordinates
    /// \return the node frame with respect to COG
    FrFrame GetFrameWRT_COG_InBody() const;

    void SetFrameInBody(const FrFrame &frameInBody);

    void SetFrameInWorld(const FrFrame &frameInWorld);


    /// Get the node position in world reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the node position in world reference frame
    Position GetPositionInWorld(FRAME_CONVENTION fc) const;

    /// Get the node position in world reference frame
    /// \param position reference to the node position in world reference frame
    /// \param fc Frame convention (NED/NWU)
    void GetPositionInWorld(Position &position, FRAME_CONVENTION fc);

    /// Get the node position in body reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the node position in body reference frame
    Position GetNodePositionInBody(FRAME_CONVENTION fc) const;

    /// Get the node velocity in world reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the node velocity in world reference frame
    Velocity GetVelocityInWorld(FRAME_CONVENTION fc) const;

    /// Get the node velocity in the node reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the node velocity in the node reference frame
    Velocity GetVelocityInNode(FRAME_CONVENTION fc) const;

    /// Get the node angular velocity in world reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the node angular velocity in world reference frame
    AngularVelocity GetAngularVelocityInWorld(FRAME_CONVENTION fc) const;

    /// Get the node angular velocity in body reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the node angular velocity in body reference frame
    AngularVelocity GetAngularVelocityInBody(FRAME_CONVENTION fc) const;

    /// Get the node angular velocity in node reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the node angular velocity in node reference frame
    AngularVelocity GetAngularVelocityInNode(FRAME_CONVENTION fc) const;

    /// Get the node acceleration in world reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the node acceleration in world reference frame
    Acceleration GetAccelerationInWorld(FRAME_CONVENTION fc) const;

    /// Get the node acceleration in the node reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the node acceleration in the node reference frame
    Acceleration GetAccelerationInNode(FRAME_CONVENTION fc) const;

    /// Initialize method
    void Initialize() override;

    void StepFinalize() override {}


    // =============================================================================================================
    // PROJECTIONS
    // =============================================================================================================

    // Projection of 3D vectors defined in FrVector.h

    /// Project a vector from node reference frame to world reference frame
    /// \tparam Vector vector template
    /// \param nodeVector vector to be projected, given in the node reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the vector projected in the world reference frame
    template<class Vector>
    Vector ProjectVectorInWorld(const Vector &nodeVector, FRAME_CONVENTION fc) const {
      return GetFrameInWorld().GetQuaternion().Rotate<Vector>(nodeVector, fc);
    }

    /// Project a vector from node reference frame to world reference frame
    /// \tparam Vector vector template
    /// \param nodeVector vector to be projected, given in the node reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the vector projected in the world reference frame
    template<class Vector>
    Vector &ProjectVectorInWorld(Vector &nodeVector, FRAME_CONVENTION fc) const {
      nodeVector = GetFrameInWorld().GetQuaternion().Rotate<Vector>(nodeVector, fc);
      return nodeVector;
    }

    /// Project a vector from world reference frame to node reference frame
    /// \tparam Vector vector template
    /// \param worldVector vector to be projected, given in the world reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the vector projected in the node reference frame
    template<class Vector>
    Vector ProjectVectorInNode(const Vector &worldVector, FRAME_CONVENTION fc) const {
      return GetFrameInWorld().GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
    }

    /// Project a vector from world reference frame to node reference frame
    /// \tparam Vector vector template
    /// \param worldVector vector to be projected, given in the world reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return the vector projected in the node reference frame
    template<class Vector>
    Vector &ProjectVectorInNode(Vector &worldVector, FRAME_CONVENTION fc) const {
      worldVector = GetFrameInWorld().GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
      return worldVector;
    }

   protected:

    void DefineLogMessages() override;


   private:

    friend std::shared_ptr<internal::FrMarker> internal::GetChronoMarker(std::shared_ptr<FrNode> node);

    friend void FrLink::SetNodes(FrNode *, FrNode *);

  };


}  // end namespace frydom


#endif //FRYDOM_FRNODE_H

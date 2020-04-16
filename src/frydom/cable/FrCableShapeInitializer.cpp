//
// Created by frongere on 15/02/2020.
//

#include "FrCableShapeInitializer.h"
#include "frydom/logging/FrEventLogger.h"

#include "frydom/environment/FrEnvironmentInc.h"

#include "frydom/cable/FrCableBase.h"
#include "frydom/cable/FrCatenaryLine.h"


namespace frydom {


  std::unique_ptr<FrCableShapeInitializer>
  FrCableShapeInitializer::Create(FrCableBase *cable, FrEnvironment *environment) {

    auto startNode = cable->GetStartingNode();
    auto endNode = cable->GetEndingNode();
    auto unstretchedLength = cable->GetUnstretchedLength();

    auto ocean = environment->GetOcean();

    auto startPosition = startNode->GetPositionInWorld(NWU);
    auto endPosition = endNode->GetPositionInWorld(NWU);

    double node_distance = (startPosition - endPosition).norm();

    if (node_distance > unstretchedLength) {
      // TAUT LINE
      return std::make_unique<internal::FrCableShapeInitializerTaut>(cable);

    } else {

      auto fluid_type = FLUID_TYPE::AIR;
      if (ocean->GetFreeSurface()->IsInWater(startPosition, NWU) ||
          ocean->GetFreeSurface()->IsInWater(endPosition, NWU)) {
        fluid_type = FLUID_TYPE::WATER;
      }

      // Using a catenary line to estimate if the line is in interaction with seabed by checking the lowest point
      // of this static model
      auto catenary_line = std::make_unique<FrCatenaryLine>("initialize", cable, true, fluid_type);
      catenary_line->UseForShapeInitialization(true);
      catenary_line->Initialize();

      if (catenary_line->HasSeabedInteraction()) {
        // Slack with seabed interactions
        return std::make_unique<internal::FrCableShapeInitializerSlackSeabed>(cable, environment);

      } else {
        // Only slack
        return std::make_unique<internal::FrCableShapeInitializerSlack>(cable, std::move(catenary_line));

      }
    }
  }

  FrCableShapeInitializer::FrCableShapeInitializer(FrCableBase *cable) :
      m_cable(cable) {}


  namespace internal {


    FrCableShapeInitializerTaut::FrCableShapeInitializerTaut(FrCableBase *cable) :
        FrCableShapeInitializer(cable),
        m_unit_vector((cable->GetEndingNode()->GetPositionInWorld(NWU) -
                       cable->GetStartingNode()->GetPositionInWorld(NWU)).normalized()) {
    }

    Position FrCableShapeInitializerTaut::GetPosition(const double &s, FRAME_CONVENTION fc) const {
      assert(0. <= s <= m_cable->GetUnstretchedLength());
      auto position = m_cable->GetStartingNode()->GetPositionInWorld(NWU) + s * m_unit_vector;
      if (IsNED(fc)) {
        internal::SwapFrameConvention<Position>(position);
      }
      return position;
    }

    Direction FrCableShapeInitializerTaut::GetTangent(const double &s, FRAME_CONVENTION fc) const {
      return m_unit_vector;
    }

    FrCableShapeInitializerSlack::FrCableShapeInitializerSlack(FrCableBase *cable,
                                                               std::unique_ptr<FrCatenaryLine> catenary_cable)
        :
        FrCableShapeInitializer(cable),
        m_catenary_line(std::move(catenary_cable)) {}

    Position FrCableShapeInitializerSlack::GetPosition(const double &s, FRAME_CONVENTION fc) const {
      assert(0. <= s <= m_cable->GetUnstretchedLength());
      return m_catenary_line->GetPositionInWorld(s, fc);
    }

    Direction FrCableShapeInitializerSlack::GetTangent(const double &s, FRAME_CONVENTION fc) const {
      return m_catenary_line->GetTangent(s, fc);
    }

    FrCableShapeInitializerSlackSeabed::FrCableShapeInitializerSlackSeabed(FrCableBase *cable,
                                                                           FrEnvironment *environment) :
        m_environment(environment),
        FrCableShapeInitializer(cable) {

      // FIXME: pour le moment, on est reducteur et on considere que l'interaction de cable se fait entierement sur
      // une portion limite de cable (gauche ou droite). On ne prend pas en compte le cas ou une partie milieu repose
      // sur le seabed

      // FIXME : pour le moment, on considere que origin_node est l'ancre et se situe sur le seabed...
      // C'est reducteur et il faudra ameliorer cela... (notamment verifier)
      // Prevoir egalement la bathymetrie variable dans GetPosition()...
      std::shared_ptr<FrNode> origin_node;
      std::shared_ptr<FrNode> final_node;
      if (cable->GetStartingNode()->GetBody()->IsFixedInWorld()) { // FIXME: IsFixedInWorld est-il un bon critere ?
        origin_node = cable->GetStartingNode();
        final_node = cable->GetEndingNode();
        m_reversed = false;

      } else if (cable->GetEndingNode()->GetBody()->IsFixedInWorld()) {
        origin_node = cable->GetEndingNode();
        final_node = cable->GetStartingNode();
        m_reversed = true;

      } else {
        event_logger::error("FrCableShapeInitializer", "Slack with seabed interaction", "");
        assert(false); // Pas pris en charge !!!
      }

      auto ocean = environment->GetOcean();
      auto fluid_type = FLUID_TYPE::AIR;
      if (ocean->GetFreeSurface()->IsInWater(origin_node->GetPositionInWorld(NWU), NWU) ||
          ocean->GetFreeSurface()->IsInWater(final_node->GetPositionInWorld(NWU), NWU)) {
        fluid_type = FLUID_TYPE::WATER;
      }

      // FIXME: pour le moment, le frottement n'est pas reglable et est nul... (en plus d'etre code en dur)
      double seabed_friction_coeff = 1.;

      m_catenary_line_seabed = std::make_unique<FrCatenaryLineSeabed>("initialize",
                                                                      origin_node,
                                                                      final_node,
                                                                      cable->GetProperties(),
                                                                      true,
                                                                      cable->GetUnstretchedLength(),
                                                                      fluid_type,
                                                                      seabed_friction_coeff);
      m_catenary_line_seabed->UseForShapeInitialization(true);
      m_catenary_line_seabed->Initialize();

//      m_origin_position = origin_node->GetPositionInWorld(NWU);
//      Position final_position = final_node->GetPositionInWorld(NWU);
//
//      m_horizontal_direction = final_position - m_origin_position;
//      double vertical_spreading = m_horizontal_direction.z(); // d
//
//      m_horizontal_direction.z() = 0.;
//      double horizontal_spreading = m_horizontal_direction.norm(); // h
//
//      m_horizontal_direction.normalize();
//
//      double L = m_cable->GetUnstretchedLength();
//      m_lying_distance = 0.5 * (L + horizontal_spreading -
//          (vertical_spreading * vertical_spreading) / (L - horizontal_spreading));
//
//      m_touch_down_point_position = m_origin_position + m_lying_distance * m_horizontal_direction;
//
//      m_raising_direction = (final_position - m_touch_down_point_position).normalized();

    }

    Position FrCableShapeInitializerSlackSeabed::GetPosition(const double &s, FRAME_CONVENTION fc) const {
      assert(0. <= s <= m_cable->GetUnstretchedLength());
      return m_catenary_line_seabed->GetPositionInWorld(s, fc);
//      double stmp;
//      if (m_reversed)
//        stmp = m_cable->GetUnstretchedLength() - s;
//      else {
//        stmp = s;
//      }
//
//      Position position;
//      if (stmp <= m_lying_distance) {
//        position = m_origin_position + stmp * m_horizontal_direction;
//      } else {
//        position = m_touch_down_point_position + (stmp - m_lying_distance) * m_raising_direction;
//      }
//
//      return position;
    }

    Direction FrCableShapeInitializerSlackSeabed::GetTangent(const double &s, FRAME_CONVENTION fc) const {
      return m_catenary_line_seabed->GetTangent(s, fc);
    }

  }  // end namespace frydom::internal
}  // end namespace frydom

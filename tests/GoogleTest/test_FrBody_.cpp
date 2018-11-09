//
// Created by Lucas Letournel on 05/11/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;


inline Position & EasyRotate(Position & vector) {
    Position res(vector.GetZ(),vector.GetX(),vector.GetY());
//    Position res(vector.GetX(),-vector.GetZ(),vector.GetY());
    return vector = res;
}

inline Translation & EasyRotate(Translation & vector) {
    Translation res(vector.GetDz(),vector.GetDx(),vector.GetDy());
//    Translation res(vector.GetDx(),-vector.GetDz(),vector.GetDy());
    return vector = res;
}

inline Velocity & EasyRotate(Velocity & vector) {
    Velocity res(vector.GetVz(),vector.GetVx(),vector.GetVy());
//    Velocity res(vector.GetVx(),-vector.GetVz(),vector.GetVy());
    return vector = res;
}

void Test_AllGetPosition(const std::shared_ptr<FrBody_> body,
        const Position &RefPositionInWorld, const Position &COGPositionInWorld,
        bool is_Orientation = false){

    // Test body reference frame position in world reference frame
    Position testPosition = body->GetPosition(NWU) - RefPositionInWorld;
    EXPECT_TRUE(testPosition.isZero());

    //-----------------COG-----------------//
    // Test COG position in world reference frame
    testPosition = body->GetCOGPositionInWorld(NWU) - COGPositionInWorld;
    EXPECT_TRUE(testPosition.isZero());
    // Test COG position in body reference frame
    Position TempPos = COGPositionInWorld - RefPositionInWorld;
//    if (is_Orientation) TempPos = EasyRotate(TempPos);
    testPosition = body->GetCOG(NWU) - TempPos;
    EXPECT_TRUE(testPosition.isZero());

    //-----------------Fixed Point-----------------//
    // Test for the getter for the local position of a point expressed in the world reference frame
    Position PointPositionInWorld(1., 5., 9.); // Position of a point, expressed in world reference frame
    TempPos = body->GetPointPositionInBody(PointPositionInWorld, NWU);
    if (is_Orientation) TempPos = EasyRotate(TempPos);
    testPosition = TempPos - (PointPositionInWorld - RefPositionInWorld);
    EXPECT_TRUE(testPosition.isZero());

    Position PointPositionInBody(1.,5.,9.); // Position of a point, expressed in body reference frame
    // Test for the getter for the abs position of a point expressed in the body reference frame
    TempPos = PointPositionInBody;
    if (is_Orientation) TempPos = EasyRotate(TempPos);
    testPosition = body->GetPointPositionInWorld(PointPositionInBody, NWU) - RefPositionInWorld - TempPos;
    EXPECT_TRUE(testPosition.isZero());

}

TEST(FrBodyTest,TestPosition) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position RefPositionInWorld(1., 2., 3.);
    body->SetPosition(RefPositionInWorld, NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position COGPositionInBody(2., 3., 4.);
    body->SetCOG(COGPositionInBody, NWU);
    Position COGPositionInWorld = RefPositionInWorld + COGPositionInBody;

    Test_AllGetPosition(body,RefPositionInWorld,COGPositionInWorld);
}

TEST(FrBodyTest,TestTranslation) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position RefPositionInWorld(1., 2., 3.);
    body->SetPosition(RefPositionInWorld, NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position COGPositionInBody(2., 3., 4.);
    body->SetCOG(COGPositionInBody, NWU);
    Position COGPositionInWorld = RefPositionInWorld + COGPositionInBody;

    //-----------------Translate Body-----------------//
    Translation BodyTranslationInWorld(1.,4.,7.);
    Position BodyPosition = RefPositionInWorld + BodyTranslationInWorld;
    Position COGPosition = COGPositionInWorld + BodyTranslationInWorld;

    //+++++Translate body reference frame+++++//
//    std::cout<<"+++++SetPosition"<<std::endl;
    body->SetPosition(RefPositionInWorld+BodyTranslationInWorld,NWU);
    Test_AllGetPosition(body,BodyPosition,COGPosition);

    //+++++Translate body reference frame from fixed point+++++//
//    std::cout<<"+++++SetPointPosition"<<std::endl;
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    Position Point(4.,5.,6.); // Position of a point expressed in body reference frame
    body->SetPointPosition(Point, body->GetPointPositionInWorld(Point,NWU) + BodyTranslationInWorld, NWU);
    Test_AllGetPosition(body,BodyPosition,COGPosition);

    //+++++Translate body reference frame from translation expressed in world reference frame+++++//
//    std::cout<<"+++++TranslateInWorld"<<std::endl;
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    body->TranslateInWorld(BodyTranslationInWorld,NWU);
    Test_AllGetPosition(body,BodyPosition,COGPosition);

    //+++++Translate body reference frame from translation expressed in body reference frame+++++//
//    std::cout<<"+++++TranslateInBody"<<std::endl;
    BodyPosition += BodyTranslationInWorld;
    COGPosition += BodyTranslationInWorld;
    body->TranslateInBody(BodyTranslationInWorld,NWU);
    Test_AllGetPosition(body,BodyPosition,COGPosition);


//    //+++++Translate body reference frame from COG+++++//
//    body->SetCOGPosition(body->GetCOGPositionInWorld(NWU)+BodyTranslationInWorld,NWU);
//    Test_AllGetPosition(body,RefPositionInWorld,COGPositionInWorld);

}

void Test_AllGetRotation(const std::shared_ptr<FrBody_> body, const FrRotation_ &BodyRotationInWorld){

    // Test of the getter for the absolute orientation (expressed in the world reference frame)
    EXPECT_TRUE(body->GetRotation() == BodyRotationInWorld);

    // Test of the getter for the quaternion
    EXPECT_TRUE(body->GetQuaternion() == BodyRotationInWorld.GetQuaternion());

    // Test of the frame getter
    EXPECT_TRUE(body->GetFrame().GetRotation() == BodyRotationInWorld);
}


TEST(FrBodyTest,TestOrientation) {
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position RefPositionInWorld(1., 2., 3.);
    body->SetPosition(RefPositionInWorld, NWU);

    //-----------------Orientation-----------------//
    //+++++Set Rotation+++++//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    FrRotation_ BodyRotationInWorld;    BodyRotationInWorld.SetCardanAngles_DEGREES(1., 2., 3., NWU);
    body->SetRotation(BodyRotationInWorld);

    Test_AllGetRotation(body, BodyRotationInWorld);

    //+++++Set Rotation, using Cardan angles+++++//
    body->SetCardanAngles_DEGREES(1., 2., 3., NWU);
    EXPECT_TRUE(BodyRotationInWorld == body->GetRotation());

    Test_AllGetRotation(body, BodyRotationInWorld);
    // Test of the frame position getter
    Position testPosition = body->GetFrame().GetPosition(NWU) - RefPositionInWorld;
    EXPECT_TRUE(testPosition.isZero());

    //+++++Set Frame+++++//
    FrFrame_ RefFrame(RefPositionInWorld,BodyRotationInWorld,NWU);
    body->SetFrame(RefFrame);

    Test_AllGetRotation(body, BodyRotationInWorld);
    // Test of the frame position getter
    testPosition = body->GetFrame().GetPosition(NWU) - RefFrame.GetPosition(NWU);
    EXPECT_TRUE(testPosition.isZero());

}

TEST(FrBodyTest,TestPositionWithOrientation){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigAbsPos(1.,2.,3.);
    body->SetPosition(OrigAbsPos,NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    body->SetCOG(OrigLocalCOGPos, NWU);
    Position OrigAbsCOGPos = OrigAbsPos + OrigLocalCOGPos;

    //-----------------Orientation-----------------//
    // Rotation to an easy transformation
    FrRotation_ Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation_ Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation_ TotalRotation = Rotation1*Rotation2 ;
    body->SetRotation(TotalRotation);


    Test_AllGetPosition(body, OrigAbsPos, OrigAbsCOGPos, true);

}

void Test_AllGetVelocity(const std::shared_ptr<FrBody_> body,
                         const Velocity &VelocityToCompare,
                         bool is_Rotation) {

    Velocity RotVelocity = VelocityToCompare;
    if (is_Rotation) RotVelocity = EasyRotate(RotVelocity);


    // Test getter for the body velocity, expressed in the world reference frame
    Velocity testVelocity;
    testVelocity = body->GetVelocityInWorld(NWU) - VelocityToCompare;
    EXPECT_TRUE(testVelocity.isZero());

    // Test getter for the body velocity, expressed in the body reference frame
    testVelocity = body->GetVelocityInBody(NWU) - RotVelocity;
    EXPECT_TRUE(testVelocity.isZero());


    //-----------------COG Velocity-----------------//
    // Test Getter for the COG velocity expressed in the world reference frame
    testVelocity = body->GetCOGVelocityInWorld(NWU) - VelocityToCompare;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the COG velocity expressed in the body reference frame
    testVelocity = body->GetCOGVelocityInBody(NWU) - RotVelocity;
    EXPECT_TRUE(testVelocity.isZero());


    //-----------------Point-----------------//
    Position Point(5.,6.,7.);
    // Test Getter for the velocity expressed in the world reference frame, at a Point expressed in body reference frame
    body->GetVelocityInWorldAtPointInBody(testVelocity, NWU);
    testVelocity -= VelocityToCompare;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the velocity expressed in the world reference frame, at a Point expressed in world reference frame
    body->GetVelocityInWorldAtPointInWorld(testVelocity, NWU);
    testVelocity -= VelocityToCompare;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the velocity expressed in the body reference frame, at a Point expressed in body reference frame
    body->GetVelocityInBodyAtPointInBody(testVelocity, NWU);
    testVelocity -= RotVelocity;
    EXPECT_TRUE(testVelocity.isZero());

    // Test Getter for the velocity expressed in the body reference frame, at a Point expressed in world reference frame
    body->GetVelocityInBodyAtPointInWorld(testVelocity, NWU);
    testVelocity -= RotVelocity;
    EXPECT_TRUE(testVelocity.isZero());
}

TEST(FrBodyTest,TestTranslationalVelocity){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos,NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in the body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    body->SetCOG(OrigLocalCOGPos, NWU);

//    //-----------------Velocity Setters-----------------//
//    //+++++Frame Velocity Setters+++++//
//    // Set the body velocity, expressed in the world reference frame
//    Velocity VelocityInWorld(1.,1.,1.);    Velocity testVelocity;
//    body->SetVelocityInWorld(VelocityInWorld,NWU);
//    Test_AllGetVelocity(body,VelocityInWorld, false);
//
//    // Set the body velocity, expressed in the body reference frame
//    body->SetVelocityInBody(VelocityInWorld,NWU);
//    Test_AllGetVelocity(body,VelocityInWorld, false);
//
//    //+++++COG Velocity Setters+++++//
//    // Test Setter for the COG Velocity expressed in the world reference frame
//    Velocity COGVelocityInWorld(0.,1.,0.);
//    body->SetCOGAbsVelocity(COGVelocityInWorld,NWU);
//    Test_AllGetVelocity(body,COGVelocityInWorld, false);
//
//    // Test Setter for the COG Velocity expressed in the body reference frame
//    Velocity COGVelocityInBody(0.,1.,0.);
//    body->SetCOGLocalVelocity(COGVelocityInBody,NWU);
//    Test_AllGetVelocity(body,COGVelocityInBody, false);
//
//    //+++++Point Velocity Setters+++++//
//    Position PointInWorld(5.,6.,7.);
//    Position PointInBody = body->GetPointPositionInBody(PointInWorld,NWU);
//
//    // Test Setter for the generalized velocity expressed in the world reference frame,
//    // at a point expressed in world reference frame
//    Velocity VelocityInWorldAtPointInWorld(0.,1.,0.);
//    body->SetGeneralizedVelocityInWorldAtPointInWorld(PointInWorld,VelocityInWorldAtPointInWorld,,NWU);
//
//    // Test Setter for the generalized velocity expressed in the world reference frame,
//    // at a point expressed in body reference frame
//    Velocity VelocityInWorldAtPointInBody(0.,1.,0.);
//    body->SetGeneralizedVelocityInWorldAtPointInBody(PointInBody,VelocityInWorldAtPointInBody,,NWU);
//
//    // Test Setter for the generalized velocity expressed in the body reference frame,
//    // at a point expressed in world reference frame
//    Velocity VelocityInBodyAtPointInWorld(0.,1.,0.);
//    body->SetGeneralizedVelocityInBodyAtPointInWorld(PointInWorld,VelocityInBodyAtPointInWorld,,NWU);
//
//    // Test Setter for the generalized velocity expressed in the body reference frame,
//    // at a point expressed in body reference frame
//    Velocity VelocityInBodyAtPointInBody(0.,1.,0.);
//    body->SetGeneralizedVelocityInBodyAtPointInBody(PointInBody,VelocityInBodyAtPointInBody,,NWU);

}


TEST(FrBodyTest,TestTranslationalVelocityWithOrientation){
    // Body Instantiation
    auto body = std::make_shared<FrBody_>();

    Position OrigWorldPos(1.,2.,3.);
    body->SetPosition(OrigWorldPos,NWU);

    //-----------------COG-----------------//
    // Set the COG position, expressed in local body reference frame
    Position OrigLocalCOGPos(2.,3.,4.);
    body->SetCOG(OrigLocalCOGPos, NWU);

    //-----------------Orientation-----------------//
    // Set a new orientation for the body, expressed using CARDAN angles, in the world reference frame)
    // Rotation to an easy transformation (X = z, Y = x, Z = y)
    FrRotation_ Rotation1; Rotation1.SetCardanAngles_DEGREES(90.,0.,0.,NWU);
    FrRotation_ Rotation2; Rotation2.SetCardanAngles_DEGREES(0.,90.,0.,NWU);
    FrRotation_ TotalRotation = Rotation1*Rotation2;
    body->SetRotation(TotalRotation);

//    //-----------------Velocity Setters-----------------//
//    //+++++Frame Velocity Setters+++++//
//    // Set the body velocity, expressed in the world reference frame
//    Velocity VelocityInWorld(1.,1.,1.);    Velocity testVelocity;
//    body->SetVelocityInWorld(VelocityInWorld,NWU);
//    Test_AllGetVelocity(body, VelocityInWorld, true);
//
//    // Set the body velocity, expressed in the body reference frame
//    body->SetVelocityInBody(VelocityInWorld,NWU);
//    Test_AllGetVelocity(body, VelocityInWorld, true);
//
//    //+++++COG Velocity Setters+++++//
//    // Test Setter for the COG Velocity expressed in the world reference frame
//    Velocity COGVelocityInWorld(0.,1.,0.);
//    body->SetCOGAbsVelocity(COGVelocityInWorld,NWU);
//    Test_AllGetVelocity(body, COGVelocityInWorld, true);
//
//    // Test Setter for the COG Velocity expressed in the body reference frame
//    Velocity COGVelocityInBody(0.,1.,0.);
//    body->SetCOGLocalVelocity(COGVelocityInBody,NWU);
//    Test_AllGetVelocity(body, COGVelocityInBody, true);


}
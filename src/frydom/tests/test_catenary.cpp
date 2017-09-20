//
// Created by frongere on 03/08/17.
//

//#include "frydom/catenary/FrCatenaryNode.h"
#include <frydom/catenary/FrCatenaryLine.h>
#include "frydom/misc/FrLinspace.h"

#include "frydom/core/FrCore.h"

using namespace frydom;
//using namespace environment;

int main(int argc, char* argv[]) {

    // Creating a support body for nodes
    auto myBody = FrBody();

    // Creating two nodes
    auto node1 = myBody.CreateNode(chrono::ChVector<>());
    auto node2 = myBody.CreateNode(chrono::ChVector<>(100, 0, 0));

    // Line properties
    double Lu = 220;
    auto u = chrono::ChVector<double>(0, sqrt(3)*0.5, 0.5);
    double q = 616.538;
    double EA = 1.5708e9;

    auto line = FrCatenaryLine(node1, node2, true, EA, Lu, q, u);

    line.solve();

    myBody.UpdateForces(false);


    auto t0 = line.get_tension(0.);
    std::cout << t0[0] << std::endl;
    std::cout << t0[1] << std::endl;
    std::cout << t0[2] << std::endl;



//    auto xx = linspace(150, 230, 1000);
//
//    double x = 150;
//    double dx = 1.;
//    while (x < 300) {
//        std::cout << std::endl <<  "Solution pour x = " << x << std::endl;
//
//        node2->SetPos(x, 0., 0.);
//        line.solve();
//
//        std::cout << line.get_cable_length() << std::endl;
//
//        x += dx;
//    }

    return 0;
}
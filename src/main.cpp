#include <Simbody.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <chrono>
#include <thread>
#include <fstream>

int main() {

    SimTK::MultibodySystem system;
    SimTK::SimbodyMatterSubsystem matter(system);

    SimTK::Body::Rigid sphereBody;
    sphereBody.addDecoration(SimTK::Transform(), SimTK::DecorativeSphere(SimTK::Real(0.1)).setColor(SimTK::Red));

    SimTK::MobilizedBody::Free freeSphere(matter.updGround(), sphereBody);

    SimTK::Visualizer viz(system);
    viz.setShowFrameRate(true);
    viz.setBackgroundType(SimTK::Visualizer::BackgroundType::SolidColor);
    viz.setBackgroundColor(SimTK::Black);
    viz.setMode(SimTK::Visualizer::Mode::RealTime);
    viz.setShutdownWhenDestructed(true);
    viz.setDesiredBufferLengthInSec(0);
    system.realizeTopology();
    SimTK::State state = system.getDefaultState();
    freeSphere.setQ(state, SimTK::Vec7(0, 0, 0, 1, 1, 1, 0));

    system.realize(state);
    viz.report(state);
    std::ofstream outF("dumpStats.txt");
    viz.setDesiredFrameRate(200);
    unsigned nFrames(0), frameCounter(0);
    unsigned timeStepMilliseconds = 5;
    auto start = std::chrono::system_clock::now();
    while (1) {

        double x(sin(nFrames*timeStepMilliseconds*1e-3));
        double y(cos(nFrames*timeStepMilliseconds*1e-3));
        double z((x + y) / 2);
        freeSphere.setQToFitTranslation(state, SimTK::Vec3(x, y, z));
        //    viz.report(state);
        viz.drawFrameNow(state);
        viz.dumpStats(outF);
        ++frameCounter;
        ++nFrames;
        //   std::this_thread::sleep_for(std::chrono::milliseconds(timeStepMilliseconds));

        if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start)) > std::chrono::milliseconds(1000)) {
            std::cout << "\n" << frameCounter << "\n";
            start = std::chrono::system_clock::now();
            frameCounter = 0;
        }
    }

    return 0;
}
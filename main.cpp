/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mola_input_kitti_dataset/KittiOdometryDataset.h>
#include <mola_metric_maps/DualVoxelPointCloud.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <chrono>
#include <iostream>
#include <thread>

/* Program usage:
 *
 * KITTI_BASE_DIR=/path/to/kitti/dataset \
 * KITTI_SEQ=00 \
 * ./kitti-dualvoxelmap-demo 0.10
 *
 */

mrpt::system::CTimeLogger profiler;

// ------------------------------------------------------
//				TestVoxelMapFromKitti
// ------------------------------------------------------
void TestVoxelMapFromKitti(
	double VOXELMAP_RESOLUTION, double VOXELMAP_MAX_RANGE)
{
	// use the MOLA module to easily load the KITTI dataset in C++:
	mola::KittiOdometryDataset kittiDataset;

	const auto kittiCfg = mola::Yaml::FromText(mola::parse_yaml(
		R""""(	
    params:
      base_dir: ${KITTI_BASE_DIR}
      sequence: ${KITTI_SEQ}
      time_warp_scale: 1.0
      publish_lidar: true
      publish_image_0: true
      publish_image_1: true
      publish_ground_truth: true
)""""));

	kittiDataset.initialize(kittiCfg);

	// ground truth trajectory:
	mrpt::poses::CPose3DInterpolator gt =
		kittiDataset.getGroundTruthTrajectory();

	std::cout << "Ground truth trajectory has: " << gt.size() << " poses.\n";

	// ----------------------
	// Voxel map
	// ----------------------
	mola::DualVoxelPointCloud map(VOXELMAP_RESOLUTION, VOXELMAP_RESOLUTION);

	// map.insertionOptions.max_range = VOXELMAP_MAX_RANGE;  // [m]
	// map.insertionOptions.ray_trace_free_space = false;	// only occupied

	// map.insertionOptions.prob_hit = 0.53;
	map.renderOptions.point_size = 2.0f;

	// gui and demo app:
	mrpt::gui::CDisplayWindow3D win("KITTI VoxelMap demo", 800, 600);

	auto glVoxelMap = mrpt::opengl::CSetOfObjects::Create();

	// create GL visual objects:
	auto glVehicle = mrpt::opengl::CSetOfObjects::Create();
	glVehicle->insert(mrpt::opengl::stock_objects::CornerXYZSimple(1.0));
	auto glObsPts = mrpt::opengl::CPointCloudColoured::Create();
	glVehicle->insert(glObsPts);

	mrpt::opengl::Viewport::Ptr glViewRGB;

	win.setCameraZoom(80.0f);

	{
		mrpt::opengl::Scene::Ptr& scene = win.get3DSceneAndLock();

		{
			auto gl_grid =
				mrpt::opengl::CGridPlaneXY::Create(-200, 200, -200, 200, 0, 10);
			gl_grid->setColor_u8(mrpt::img::TColor(0x80, 0x80, 0x80));
			scene->insert(gl_grid);
		}
		scene->insert(mrpt::opengl::stock_objects::CornerXYZSimple(3.0));

		scene->insert(glVehicle);

		scene->insert(glVoxelMap);

		glViewRGB = scene->createViewport("rgb_view");
		glViewRGB->setViewportPosition(0, 0.7, 0.3, 0.25);
		glViewRGB->setTransparent(true);

		win.unlockAccess3DScene();
	}

	std::cout << "Close the window to exit" << std::endl;

	size_t datasetIndex = 0;
	bool paused = false;

	mrpt::Clock::time_point lastObsTim;

	while (win.isOpen())
	{
		win.get3DSceneAndLock();

		// Get and process one observation:
		if (datasetIndex < gt.size() && !paused)
		{
			mrpt::obs::CObservationPointCloud::Ptr obs =
				kittiDataset.getPointCloud(datasetIndex);
			if (!obs) break;

			lastObsTim = obs->timestamp;

			auto gtPoseIt = gt.begin();
			std::advance(gtPoseIt, datasetIndex);

			mrpt::math::TPose3D gtPose = gtPoseIt->second;

			// win.setCameraPointingToPoint(gtPose.x, gtPose.y, gtPose.z);

			// set viz camera pose:
			glVehicle->setPose(gtPose);

			// draw observation raw data:
			glObsPts->loadFromPointsMap(obs->pointcloud.get());
			glObsPts->setPose(obs->sensorPose);

			// update the voxel map:
			mrpt::system::CTimeLoggerEntry tle1(profiler, "insertObservation");

			map.insertObservation(*obs, mrpt::poses::CPose3D(gtPose));

			tle1.stop();

			// Update the voxel map visualization:
			static int decimUpdateViz = 0;
			if (decimUpdateViz++ > 40)
			{
				decimUpdateViz = 0;
				glVoxelMap->clear();

				mrpt::system::CTimeLoggerEntry tle2(
					profiler, "getVisualizationInto");

				map.getVisualizationInto(*glVoxelMap);
			}

			// RGB view:
			auto obsIm = kittiDataset.getImage(0, datasetIndex);
			if (obsIm) { glViewRGB->setImageView(obsIm->image); }

			datasetIndex++;
		}

		win.unlockAccess3DScene();

		if (win.keyHit())
		{
			const unsigned int k = win.getPushedKey();

			switch (k)
			{
				case ' ': paused = !paused; break;
			};
		}

		win.addTextMessage(
			5, 5,
			mrpt::format(
				"Timestamp: %s datasetIndex: %zu",
				mrpt::system::dateTimeLocalToString(lastObsTim).c_str(),
				datasetIndex),
			1 /*id*/);

		win.repaint();

		using namespace std::chrono_literals;
		std::this_thread::sleep_for(10ms);
	};
}

int main(int argc, char** argv)
{
	try
	{
		if (argc != 2 && argc != 3)
			throw std::invalid_argument(
				"Usage: PROGRAM <VOXELMAP_RESOLUTION> [<VOXELMAP_MAX_RANGE>]");

		double VOXELMAP_MAX_RANGE = 90.0;
		if (argc == 3) { VOXELMAP_MAX_RANGE = std::stod(argv[2]); }

		TestVoxelMapFromKitti(std::stod(argv[1]), VOXELMAP_MAX_RANGE);

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception: " << e.what() << std::endl;
		return -1;
	}
}

#include "Functions.h"
#include <queue>
#include <mutex>

////滤波处理
//enum PointCloudProcessMode{Capture, Process};
////先是数据读入状态，接收【当前机械臂位姿】信号
////bool isBusy = false;//是否正在处理点云。
//PointCloudProcessMode SystemMode = Capture;

//PCXYZ_Ptr PointsOfTable(new PCXYZ);//合成后的桌子的点云。
//std::queue<PCXYZ_Ptr> queue_PointsOfCapture;//被捕获的点云的存放队列。
//std::queue<Eigen::Matrix4f> queue_TransformData;//被捕获的点云的位姿矩阵。
//std::mutex QueueMutex;
//// 作为主动触发的函数
//std::string Capture_PointClould(double TransformData[4][4])/****RTM****/
//{
//	PCXYZ_Ptr DataIn = PCXYZ_Ptr(new PCXYZ());//到时候换成RTM点云输入变量。

//	if (!QueueMutex.try_lock())
//		std::cout << "Processing!" << std::endl;
//	while (!QueueMutex.try_lock());
//	//isBusy = true;

//	queue_PointsOfCapture.push(DataIn);
//	queue_TransformData.push(MakeTransformMatrix(TransformData));

//	QueueMutex.unlock();

//	return "Capture_PointClould() Success!";
//}

////应该在OnExecute函数内，在 拍摄 模式时，每周期调用。
//void Transform_PointCloud()
//{
//	if (SystemMode != Capture)
//		return;

//	if (!QueueMutex.try_lock())
//		return;
//	//isBusy = true;//防止队列（在RTM中）的多线程调用冲突。

//	if (queue_PointsOfCapture.empty() || queue_TransformData.empty())
//	{
//		QueueMutex.unlock();
//		return;
//	}

//	PCXYZ_Ptr tmp(new PCXYZ);
//	PCXYZ_Ptr tmp2(new PCXYZ);
//	PCXYZ_Ptr tmp3(new PCXYZ);

//	pcl::transformPointCloud(*queue_PointsOfCapture.front(), *tmp, queue_TransformData.front());

//	//调用完后弹出队列
//	queue_PointsOfCapture.pop();
//	queue_TransformData.pop();

//	QueueMutex.unlock();//关闭忙标志，使能队列操作。

//	Filters(tmp, tmp2);

//	//ICP
//	if (PointsOfTable.size() != 0)
//		ICP_Single(tmp2, PointsOfTable, tmp3);

//	*PointsOfTable += *tmp2;//累加点云
//}

////主动触发
//std::string Clear_QueueAndPoints()
//{
//	if (!QueueMutex.try_lock())
//		return "Processing!";
//	//isBusy = true;

//	(*PointsOfTable).clear();

//	while (!queue_PointsOfCapture.empty())
//		queue_PointsOfCapture.pop();
//	while (!queue_TransformData.empty())
//		queue_TransformData.pop();

//	QueueMutex.unlock();

//	return "Clear_Queue_Points() Success!";
//}

////主动触发
//std::string SwitchSysMode(std::string ModeStr)
//{
//	switch (SystemMode)
//	{
//		case Capture://当前模式
//			switch (ModeStr)//目标模式
//			{
//				case "CaptureMode":
//					return "Now CaptureMode. No need to switch!"
//					break;
//				case "ProcessMode":
//					if (!queue_PointsOfCapture.empty())
//						return "Now CaptureMode. Processing......!";
//					SystemMode = Process;
//					break;
//				default:
//					break;
//			}
//			break;
//		case Process:
//			switch (ModeStr)//目标模式
//			{
//				case "CaptureMode":
//					SystemMode = Capture;
//					Clear_QueueAndPoints();
//					return "Switch to Capture mode and clear Queue and GlobePointCloud successfully!";
//					break;
//				case "ProcessMode":
//					return "Now ProcessMode. No need to switch!"
//					break;
//				default:
//					break;
//			}
//			break;
//		default:
//			break;
//	}
	
//}



Eigen::Matrix4f MakeTransformMatrix(double Data[4][4])
{
	Eigen::Matrix4f tmp;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			tmp(i, j) = Data[i][j];
	return tmp;
}

PCXYZ_Ptr Filters(PCXYZ_Ptr Source, PCXYZ_Ptr Output)
{
	//////// Settings of PassFilter
	pcl::PassThrough<pcl::PointXYZ> PassFilter;
	PassFilter.setFilterFieldName("z");
	PassFilter.setFilterLimits(0.020, 0.50);

	//////// Settings of PassFilter
	pcl::VoxelGrid<pcl::PointXYZ> VoxelGrid_sor;
	VoxelGrid_sor.setLeafSize(0.005f, 0.005f, 0.005f);//Set the size of VoxelGrid.

	//////// Settings of BilateraFilter
	pcl::FastBilateralFilter<pcl::PointXYZ> fbf;
	fbf.setSigmaS(1);
	fbf.setSigmaR(0.02);

	//////// Settings of StatisticalOutlierRemover
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setMeanK(50);
	sor.setStddevMulThresh(0.6);

	//////// Temporary variables for this function.
	PCXYZ_Ptr tmp(new PCXYZ);
	PCXYZ_Ptr tmp2(new PCXYZ);
	PCXYZ_Ptr tmp3(new PCXYZ);

	//////// Do the FastBilateralFilter.
	CovertTo_OrgnizedPointCloud(Source, 640, 480);
	fbf.setInputCloud(Source);
	fbf.filter(*tmp);
	CovertTo_UnOrgnizedPointCloud(tmp);

	//////// Do the VoxelGrid filter.
	VoxelGrid_sor.setInputCloud(tmp);
	VoxelGrid_sor.filter(*tmp2);

	//////// Do the Pass filter.
	PassFilter.setInputCloud(tmp2);
	PassFilter.filter(*tmp3);

	//////// Do the
	sor.setInputCloud(tmp3);
	sor.filter(*Output);

	return Output;
}

void CovertTo_OrgnizedPointCloud(PCXYZ_Ptr &Source, double Width, double Height)
{
	/**/////原始点云大小(Size)=Height*Width S=H*W Scale=W/H
		//S=H*H*Scale
		//H=sqrt(S/Scale)
	double Scale = Width / Height;
	double RealHeight = sqrt((*Source).size() / Scale);

	(*Source).height = round(RealHeight);///注意，不能让新算的总点数超过真实点数。
	(*Source).width = RealHeight * Scale;
	if ((*Source).height * (*Source).width > (*Source).size())//如果新算的大小大于原始的，则将宽度减少1；
		(*Source).width--;
}

void CovertTo_UnOrgnizedPointCloud(PCXYZ_Ptr &Source)
{
	(*Source).height = 1;
	(*Source).width = (*Source).size();
}

void ExtractPlane(PCXYZ_Ptr Source, PCXYZ_Ptr Plane, PCXYZ_Ptr Rest)//抽出平面，Rest去除平面后的点云；返回值是抽出的平面。
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);// Coefficients for extracted model.
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);// points of extracted model.
	pcl::SACSegmentation<pcl::PointXYZ> seg;// Create the segmentation(Plane extract) object
	pcl::ExtractIndices<pcl::PointXYZ> extract;//For converting pcl::PointIndices to PointCloud

	//////// Settings of SACSegmentation.
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.02);

	//////// Do extract.
	seg.setInputCloud(Source);
	seg.segment(*inliers, *coefficients);


	//////// Output process
	if (inliers->indices.size() == 0)
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
	extract.setInputCloud(Source);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*Plane);// Get the points associated with the planar surface.
	extract.setNegative(true);
	extract.filter(*Rest);// Remove the planar inliers, extract the rest.
}

int ExtractEuclideanCluster(PCXYZ_Ptr Source, PCXYZ_Ptr Clusters[])
{
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(Source);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.01); // 2cm
	ec.setMinClusterSize(300);
	ec.setMaxClusterSize(450000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(Source);
	ec.extract(cluster_indices);

	int i = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(Source->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		Clusters[i++] = cloud_cluster;
	}
	return cluster_indices.size();
}

Mat4f ICP_Single(PCXYZ_Ptr Source, PCXYZ_Ptr Target, PCXYZ_Ptr Output)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	icp.setMaxCorrespondenceDistance(1);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.001);

	icp.setMaximumIterations(300);
	icp.setInputSource(Source);
	icp.setInputTarget(Target);//这个点云不动，其他点云跟Target配合。
	icp.align(*Output);

	if (icp.hasConverged())
	{
		std::cout << "ICP has converged, score is " << icp.getFitnessScore();
		pcl::transformPointCloud(*Source, *Output, icp.getFinalTransformation());
		return icp.getFinalTransformation();
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return Mat4f().setZero();
	}
}

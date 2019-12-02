#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <vtkSmartPointer.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace pcl;
using namespace std;
using namespace cv;
using namespace pcl::io;
using namespace pcl::console;

#define presentationMode false

struct Camera {
    float focalLengthX;
    float focalLengthY;
    float centerX;
    float centerY;
};

bool isNumm(const std::string &s) {
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}

// file name have to be like: "smth_num_smth.png"
string getFileId(string str) {
    std::string delimiter = "_";
    size_t pos = 0;
    std::string token;
    while ((pos = str.find(delimiter)) != std::string::npos) {
        token = str.substr(0, pos);
        str.erase(0, pos + delimiter.length());
        if (isNumm(token))
            return token;
    }
}

vector<string> loadImageFiles(const char *path) {
    vector<string> filesList;
    DIR *dir;
    struct dirent *ent;

    if ((dir = opendir(path))) {
        while ((ent = readdir(dir)) != NULL) {
            string fileType = ent->d_name;
            if (fileType.find(".")) {
                filesList.push_back(ent->d_name);
            }
        }
        closedir(dir);
    } else {
        perror("It is not possible to open given folder!");
    }
    std::sort(filesList.begin(), filesList.end());
    return filesList;
}

pcl::PointCloud<pcl::PointXYZRGB> makePointCloud(Mat textureImg, Mat depthMat, Camera camera) {
    int width = textureImg.cols;
    int height = textureImg.rows;

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.width = width;
    cloud.height = height;
    cloud.points.resize(height * width);
    cloud.is_dense = false;

    const float scalingFactor = 1.0;

    for (int v = 0; v < width; v++) {
        for (int u = 0; u < height; u++) {
            pcl::PointXYZRGB &pt = cloud.at(v, u);
            pt.z = depthMat.at<unsigned short>(v, u * 1.5) / scalingFactor; //TODO: *1.5 - do not know why

            if (pt.z != 0) {
                pt.x = (static_cast<float> (u) - camera.centerX) * pt.z / camera.focalLengthX;
                pt.y = (static_cast<float> (v) - camera.centerY) * pt.z / camera.focalLengthY;
                pt.b = textureImg.at<cv::Vec3b>(v, u)[0];
                pt.g = textureImg.at<cv::Vec3b>(v, u)[1];
                pt.r = textureImg.at<cv::Vec3b>(v, u)[2];
            } else {
                pt.x = pt.y = pt.z = pt.r = pt.g = pt.b = std::numeric_limits<float>::quiet_NaN();
            }
            cloud.at((int) v, (int) u) = pt;
        }
    }
    return cloud;
}


int main(int argc, char **argv) {

    Camera camera;
    camera.focalLengthX = 521.29; // 1.303225*800/2
    camera.focalLengthY = 521.29; // 1.303225*800/2
    camera.centerX = 400;         // width/2
    camera.centerY = 400;         // height/2

    const char *depthFileType;
    depthFileType = "../to_pcl/depth/";
    vector<string> depthFiles = loadImageFiles(depthFileType);

    const char *textureFileType;
    textureFileType = "../to_pcl/texture/";
    vector<string> textureFiles = loadImageFiles(textureFileType);

    int numFilesToProcess;
    if (textureFiles.size() == depthFiles.size()) {
        numFilesToProcess = textureFiles.size();
    } else {
        cerr << "Count of texture files is different from depth files count. " \
 << textureFiles.size() << " vs. " << depthFiles.size() << endl;
        return -1;
    }

    cout << "Generating .ply files" << endl;
    for (int i = 0; i < 5; i++) { // i < numFilesToProcess - to process all files in directory
        cv::Mat depthMap;
        depthMap = cv::imread(string(depthFileType) + depthFiles.at(i));
        cv::Mat textureImg = cv::imread(string(textureFileType) + textureFiles.at(i));
        pcl::PointCloud<pcl::PointXYZRGB> cloud = makePointCloud(textureImg, depthMap, camera);

        // cloud ptr for visualization
        /*
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr (&cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (&cloud);
        */

        // remove outliers (because of low depths map resolution)
        /*
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (cloudPtr);
        sor.setMeanK (10);
        sor.setStddevMulThresh (1.0);
        sor.filter (filteredCloud);
        */

        // save generated .ply file
        string pclName = "pcl" + getFileId(depthFiles.at(i)) + ".ply";
        pcl::io::savePLYFileBinary("../to_pcl/pointcloud/" + pclName, cloud);
        cout << ".";

        if (presentationMode) {
            cv::imshow("Depth map", depthMap);
            cv::imshow("Texture image", textureImg);
            cv::waitKey(0);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(&cloud);

            pcl::visualization::CloudViewer viewer("Generated Point Cloud");
            viewer.showCloud(cloudPtr);

            while (!viewer.wasStopped()) {
            }
        }
    }
    return 0;
}
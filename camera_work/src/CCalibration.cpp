#include "../include/CCalibration.hpp"

CCalibration::CCalibration(cv::Size _board_sz, double _board_dt, int _n_boards)
{
	//标定板的信息
	board_sz = _board_sz;
	board_dt = _board_dt;
	n_boards = _n_boards;
 
	//为标定参数分配内存
	intrinsic_matrix  = cv::Mat(3,3,CV_32FC1);//不需要手动释放内存
	distortion_coeffs = cv::Mat(4,1,CV_32FC1);
}

/* 
* 函数名称：calibrateFromCamera
* 函数功能：直接从相机实时获取标定板图像，用于标定
* 函数入口：  
* 输入参数：五
* 输出参数：无
* 返 回 值：是否标定成功，true表示成功，false表示失败
* 其它说明：  
*/ 
bool CCalibration::calibrateFromCamera()
{
    cv::namedWindow("Calibration", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Live", cv::WINDOW_AUTOSIZE);

    cv::VideoCapture capture(0,cv::CAP_V4L2); // 将要标定的摄像头
    if (!capture.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return false;
    }

    int board_n = board_sz.width * board_sz.height; // 角点总数
    std::cout << board_n << std::endl;
    cv::Mat image_points(n_boards * board_n, 2, CV_32FC1);
    cv::Mat object_points(n_boards * board_n, 3, CV_32FC1);
    cv::Mat point_counts(n_boards, 1, CV_32SC1);

    std::vector<cv::Point2f> corners;
    cv::Mat image, gray_image;

    int corner_count;
    int successes = 0; // 图像系列index
    int step, frame = 0;

    // 忽略开始前2s时间的图片
    for (int i = 0; i < 33 * 2; i++) {
        capture >> image;
        cv::imshow("Live", image);
        cv::waitKey(30);
    }

    // 获取足够多视场图片用于标定
    while (successes < n_boards) {
        capture >> image;
        cv::imshow("Live", image);
        cv::waitKey(33); // 一帧的时间间隔

        // 每隔board_dt秒取一张图像
        if ((frame++ % ((int)(33 * board_dt))) == 0) {
            // Find chessboard corners:
            bool found = cv::findChessboardCorners(image, board_sz, corners,
                                                   cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
            if (!found) continue; // 未正确找到角点，继续下一次

            // Get Subpixel accuracy on those corners
            cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY); // 转换为灰度图像
            cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

            // 如果该视场获得了好的结果，保存它
            if (corners.size() == board_n) {
                step = successes * board_n;
                for (int i = step, j = 0; j < board_n; ++i, ++j) {
                    image_points.at<float>(i, 0) = corners[j].x;
                    image_points.at<float>(i, 1) = corners[j].y;
                    object_points.at<float>(i, 0) = j / board_sz.width;
                    object_points.at<float>(i, 1) = j % board_sz.width;
                    object_points.at<float>(i, 2) = 0.0f;
                }
                point_counts.at<int>(successes, 0) = board_n;
                successes++;
                std::cout << "Successes: " << successes << std::endl;
                std::cout << "Image points: " << image_points.rows << " Object points: " << object_points.cols << std::endl;
            }

            // Draw corners
            cv::drawChessboardCorners(image, board_sz, corners, found);
            std::string text = std::to_string(successes) + "/" + std::to_string(n_boards);
            cv::putText(image, text, cv::Point(40, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            cv::imshow("Calibration", image);
        }
    }

    // 获取了足够多视场，结束获取
    cv::destroyWindow("Calibration");
    cv::destroyWindow("Live");

    // 计算 核心函数
    doCalibrate(image_points, object_points, point_counts, image.size());

    return true;
}



/* 
* 函数名称：calibrateFromCamera
* 函数功能：根据已获取的图像文件（.bmp格式），标定相机
* 函数入口：  
* 输入参数：无
* 输出参数：无
* 返 回 值：是否标定成功，true表示成功，false表示失败
* 其它说明： 只接受.bmp格式的图片，且图片尺寸要相同，若要标定其他格式图片，请将本函数内的.bmp替换成.jpg
*            文件统一命名格式为 calib_N.bmp,其中N必须从0开始
*/  
bool CCalibration::calibrateFromFile()
{
    cv::namedWindow("Calibration", cv::WINDOW_AUTOSIZE);
    int board_n = board_sz.width * board_sz.height; // 角点总数
    cv::Mat image_points(n_boards * board_n, 2, CV_32FC1);
    cv::Mat object_points(n_boards * board_n, 3, CV_32FC1);
    cv::Mat point_counts(n_boards, 1, CV_32SC1);

    std::vector<cv::Point2f> corners;
    cv::Mat image, gray_image;

    int corner_count;
    int successes = 0, index = 0; // 图像系列index
    int step;
    std::string folderName = "imgName";
    std::cout << "Current path is " << std::filesystem::current_path() << '\n';

    // 获取足够多视场图片用于标定
    while (successes < n_boards)
    {
        std::string imgname = folderName + "/calib_" + std::to_string(index++) + ".bmp";
        image = cv::imread(imgname, cv::IMREAD_COLOR);
        std::cout << "Image path: " << imgname << std::endl;
        if (image.empty()) 
        {
            std::cout << "faild" << std::endl;
            break;
        
        } // 无此图片，则停止
        cv::waitKey(1000 * board_dt); // 一帧的时间间隔

        // Find chessboard corners:
        bool found = cv::findChessboardCorners(image, board_sz, corners,
                                               cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
        if (!found) continue; // 未正确找到角点，继续下一次

        // Get Subpixel accuracy on those corners
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY); // 转换为灰度图像
        cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

        // 如果该视场获得了好的结果，保存它
        if (corners.size() == board_n)
        {
            step = successes * board_n;
            for (int i = step, j = 0; j < board_n; ++i, ++j)
            {
                image_points.at<float>(i, 0) = corners[j].x;
                image_points.at<float>(i, 1) = corners[j].y;
                object_points.at<float>(i, 0) = j / board_sz.width;
                object_points.at<float>(i, 1) = j % board_sz.width;
                object_points.at<float>(i, 2) = 0.0f;
            }
            point_counts.at<int>(successes, 0) = board_n;
            successes++;
        }

        // Draw corners
        cv::drawChessboardCorners(image, board_sz, corners, found);
        std::string text = std::to_string(successes) + "/" + std::to_string(n_boards);
        cv::putText(image, text, cv::Point(40, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        cv::imshow("Calibration", image);
    }

    // 获取了足够多视场，结束获取
    cv::destroyWindow("Calibration");

    // 计算
    doCalibrate(image_points, object_points, point_counts, image.size());

    return true;
}



/* 
* 函数名称：doCalibrate 
* 函数功能：计算相机内参数和畸变参数 
* 函数入口：
* 输入参数：存储图像角点坐标（成像仪坐标）信息的矩阵指针image_points，存储有标定板角点坐标（世界坐标）信息的矩阵指针object_points
*			存储有各图像寻找到的角点个数信息的矩阵指针point_counts，图像尺寸size
* 输出参数：无 
* 返 回 值： 是否成功，true成功，false失败
* 其它说明： 标定结果同时存储到当前目录Intrinsics.xml，Distortion.xml文件中
*/  

bool CCalibration::doCalibrate(const cv::Mat& image_points, const cv::Mat& object_points, const cv::Mat& point_counts, const cv::Size& size)
{
    int board_n = board_sz.width * board_sz.height; // 角点总数
    std::vector<std::vector<cv::Point3f>> object_points_vec;
    std::vector<std::vector<cv::Point2f>> image_points_vec;

    for (int i = 0; i < point_counts.rows; ++i) {
        std::vector<cv::Point3f> obj_pts;
        std::vector<cv::Point2f> img_pts;
        for (int j = 0; j < point_counts.at<int>(i, 0); ++j) {
            obj_pts.push_back(cv::Point3f(object_points.at<float>(i * board_n + j, 0),
                                          object_points.at<float>(i * board_n + j, 1),
                                          object_points.at<float>(i * board_n + j, 2)));
            img_pts.push_back(cv::Point2f(image_points.at<float>(i * board_n + j, 0),
                                          image_points.at<float>(i * board_n + j, 1)));
        }
        object_points_vec.push_back(obj_pts);
        image_points_vec.push_back(img_pts);
    }
    // 计算标定参数
    std::vector<cv::Mat> rvecs, tvecs;
    cv::calibrateCamera(object_points_vec, image_points_vec, size,
                        intrinsic_matrix, distortion_coeffs, rvecs, tvecs);

    // 保存内参和畸变系数
    cv::FileStorage fs_intrinsics("Intrinsics.xml", cv::FileStorage::WRITE);
    fs_intrinsics << "intrinsic_matrix" << intrinsic_matrix;
    fs_intrinsics.release();

    cv::FileStorage fs_distortion("Distortion.xml", cv::FileStorage::WRITE);
    fs_distortion << "distortion_coeffs" << distortion_coeffs;
    fs_distortion.release();

    return true;
}



/* 
* 函数名称：display 
* 函数功能：根据标定参数，显示修正后的视频图像 
* 函数入口：
* 输入参数：无
* 输出参数：无 
* 返 回 值： 
* 其它说明：  
*/  
void CCalibration::display()
{
    cv::namedWindow("Undistort", cv::WINDOW_AUTOSIZE); // 显示修正后图像

    cv::VideoCapture capture(0,cv::CAP_V4L2);
    if (!capture.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return;
    }

    cv::Mat frame;
    capture >> frame;
    if (frame.empty()) {
        std::cerr << "Error: Could not capture frame." << std::endl;
        return;
    }

    cv::Mat imgUndistort = cv::Mat::zeros(frame.size(), frame.type());

    // 加载摄像头内参数和畸变参数
    cv::Mat intrinsic, distortion;
    cv::FileStorage fs_intrinsic("Intrinsics.xml", cv::FileStorage::READ);
    fs_intrinsic["intrinsic_matrix"] >> intrinsic;
    fs_intrinsic.release();

    cv::FileStorage fs_distortion("Distortion.xml", cv::FileStorage::READ);
    fs_distortion["distortion_coeffs"] >> distortion;
    fs_distortion.release();

    // 计算畸变映射
    cv::Mat mapx, mapy;
    cv::initUndistortRectifyMap(intrinsic, distortion, cv::Mat(), intrinsic, frame.size(), CV_32FC1, mapx, mapy);

    while (cv::waitKey(33) != 27) // ESC
    {
        capture >> frame;
        if (frame.empty()) break;

        cv::remap(frame, imgUndistort, mapx, mapy, cv::INTER_LINEAR);
        cv::imshow("Undistort", imgUndistort);
    }

    capture.release();
    cv::destroyWindow("Undistort");
}

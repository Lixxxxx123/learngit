#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tdt_interface/msg/receive_data.hpp>
#include <tdt_interface/msg/send_data.hpp>
#include <cmath>

using namespace std;
using namespace cv;

class Guide
{
public:
    Guide():edit_map(51,51,CV_8UC1){}
    struct MyPoint
    {
        int x,y; //当前位置
        int f;
        int g;
        int h;
    };

    struct TreeNode
    {
        MyPoint myself;
        vector<TreeNode*> son;
        TreeNode* father;

        TreeNode(MyPoint my):myself(my),son(),father(nullptr){}
    };

    void guide(Mat map,MyPoint beginP,MyPoint endP){
        Edit_map(map);
        qidong=xunlu(edit_map,beginP,endP);
    }

    
    Mat get_edit_map(){
        return edit_map;
    }
    
    void drawPath(Mat& image){
        DrawPath(image);
    }

    void drawNode(Mat& image,vector<MyPoint>& path_node){
        DrawNode(image,path_node);
    }


    bool qidong;
private:
    Mat edit_map;
    vector<MyPoint> path;

    void DrawPath(Mat& image){
        for(int i=1;i<path.size();i++){
            line(image,Point(path[i-1].x*25,path[i-1].y*25),Point(path[i].x*25,path[i].y*25),Scalar(0,0,255),2);
        }
    }

    void DrawNode(Mat& image , vector<MyPoint>& path_node){
        path_node.push_back(path[0]);
        for (int i = 1; i < path.size()-1; i++)
        {
            if(!((path[i-1].x==path[i].x && path[i].x==path[i+1].x )||( path[i-1].y==path[i].y && path[i].y==path[i+1].y))){
                path_node.push_back(path[i]);
            }
        }
        path_node.push_back(path[path.size()-1]);
        //倒序排列
        reverse(path_node.begin(),path_node.end());
        for(int i=0;i < path_node.size();i++){
            circle(image,Point(path_node[i].x*25,path_node[i].y*25),5,Scalar(0,255,0),-1);
        }
        path.clear();      
    }

    bool xunlu(Mat map,MyPoint beginP,MyPoint endP){
        vector<vector<bool>> isfind(51,vector<bool>(51,false));

        beginP.f=0;
        beginP.h=0;
        beginP.g=0;
        TreeNode rootP(beginP);
        isfind[beginP.y][beginP.x]=true;

        TreeNode* currentP =&rootP;
        TreeNode* childP;
        vector<TreeNode*> buff;
        bool isfindend =false;

        while (true)
        {
            //找四个点出来
            for(int i =0;i<4;i++)
            {
                childP=new TreeNode(currentP->myself);
                switch (i)
                {
                case 0:   //左
                    childP->myself.x--;
                    childP->myself.g +=10;
                    cout<<"子节点位置：（"<<childP->myself.x<<","<<childP->myself.y<<")"<<endl;
                    break;
                case 1:   //上
                    childP->myself.y--;
                    childP->myself.g +=10;
                    cout<<"子节点位置：（"<<childP->myself.x<<","<<childP->myself.y<<")"<<endl;
                    break; 
                case 2:   //右
                    childP->myself.x++;
                    childP->myself.g +=10;
                    cout<<"子节点位置：（"<<childP->myself.x<<","<<childP->myself.y<<")"<<endl;
                    break; 
                case 3:   //下
                    childP->myself.y++;
                    childP->myself.g +=10;
                    cout<<"子节点位置：（"<<childP->myself.x<<","<<childP->myself.y<<")"<<endl;
                    break;
                }
                //判断能不能走
                if(canwalk(childP->myself,edit_map,isfind))
                {
                    //计算h值
                    childP->myself.h=getH(childP->myself,endP);
                    //计算f值
                    childP->myself.f=childP->myself.h+childP->myself.g;
                    //放到数组中
                    buff.push_back(childP);
                    cout<<"当前可扩展点数"<<buff.size()<<endl;
                    childP->father=currentP;
                    isfind[childP->myself.y][childP->myself.x]=true;
                }
                else
                {
                    delete childP;
                }
            }

            //从数组中找出f最小
            TreeNode* fmin=buff[0];
            int num=0;
            for (int i =1;i<buff.size();i++)
            {
                if(buff[i]->myself.f < fmin->myself.f)
                {
                    fmin=buff[i];
                    num=i;
                }
            }

            //找不到终点
            if (buff.empty())
            {
                break;
            }
            //移动
            currentP = fmin;
            //把挑出来的从数组中删除
            buff.erase(buff.begin()+num);

            cout<<"当前点（"<<currentP->myself.x<<","<<currentP->myself.y<<")"<<endl;
            cout<<"当前可扩展点数"<<buff.size()<<endl;
            //判断是否找到终点
            if(currentP->myself.x == endP.x && currentP->myself.y == endP.y){
                isfindend=true;
                break;
            }
            
        }

        if(isfindend){
            cout<<"找到终点了"<<endl;
            while (currentP)
            {
                cout<<currentP->myself.x<<","<<currentP->myself.y<<endl;
                path.push_back(currentP->myself);
                currentP=currentP->father;
            } 
            return true;
            // drawPath(edit_map,path);

        }
        else{
            cout<<"没有找到终点"<<endl;
            return false;
        }
    }


    void Edit_map(Mat map){
        for(int y =0;y<51;y++){
            for(int x=0;x<51;x++){
                edit_map.at<uint8_t>(y,x)=map.at<uint8_t>(y*25,x*25);
            }
            // if(y=50){
            //     for(int x=0;x<51;x++){
            //         edit_map.at<uint8_t>(y,x)=255;
            //     }
            // }
        }   
    }
    
    bool canwalk(MyPoint pos,Mat map,vector<vector<bool>>& isfind){
        //越界
        if(pos.y<0 || pos.y>=map.rows-1||pos.x<0||pos.x>=map.cols){
            cout<<"越界"<<endl;
            return false;
        }
        //是障碍物
        if(map.at<uint8_t>(pos.y,pos.x)==0)
        {
            cout<<"是障碍物"<<endl;
            return false;
        }
        //走过
        if(isfind[pos.y][pos.x])
        {
            cout<<"走过"<<endl;
            return false;
        }
        return true;
    }

    int getH(MyPoint pos,MyPoint end){
        int x,y;
        if (pos.x>end.x)
        {
            x=pos.x-end.x;
        }
        else
        {
            x=end.x-pos.x;
        }
        if (pos.y>end.y)
        {
            y=pos.y-end.y;
        }
        else
        {
            y=end.y-pos.y;
        }
        return 10*(x+y);
    }
};


class Detector
{
public:
    Detector(Mat input_image) : frame(input_image),_size(0)
    {
        if (input_image.empty())
        {
            cout << "输入图像为空" << endl;
        }
    }

    // 切换识别目标的颜色
    void select(char A)
    {
        // 蓝色
        if (A == 'A' || A == 'a')
        {
            setcolorRange(hmin1, hmax1, smin1, smax1, vmin1, vmax1);
        }
        // 红色
        else
        {
            setcolorRange(hmin2, hmax2, smin2, smax2, vmin2, vmax2);
        }
    }

    // 检测
    void detect()
    {

        processImage();
        findcontours();
        Dengtiao();
        Zhuangjiaban();
    }

    // 得到装甲板数量，用于创建装甲板类
    int get_zhuangjia_num()
    {
        return _size;
    }

    // 得到装甲板位置，用于数字检测
    vector<RotatedRect> get_zhuangjia_position()
    {
        return zhuangjia_posion;
    }

    // 得到装甲板信息，用于姿态解算
    vector<vector<Point2f>> get_zhuangjia_passage()
    {
        return zhuangjiaban_sum;
    }

    Mat Imshow_frame(){
        return frame;
    }

    Mat Imshow_Dil(){
        return frameDil;
    }

private:
    Mat frame, frameHSV, frameBlur, frameDil, mask;
    vector<vector<Point>> contours;

    int hmin, hmax, vmin, vmax, smin, smax;
    int hmin1 = 66, smin1 = 0, vmin1 = 221, hmax1 = 98, smax1 = 137, vmax1 = 255;
    int hmin2 = 0, smin2 = 79, vmin2 = 152, hmax2 = 179, smax2 = 255, vmax2 = 255;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));

    vector<Point2f> centerpoints, toppoints, downpoints;
    vector<vector<Point2f>> zhuangjiaban_sum;
    int _size;
    vector<RotatedRect> zhuangjia_posion;
    vector<RotatedRect> dengtiao;

    // 设置颜色
    void setcolorRange(int hmin_n, int hmax_n, int smin_n, int smax_n, int vmin_n, int vmax_n)
    {
        hmin = hmin_n;
        hmax = hmax_n;
        smin = smin_n;
        smax = smax_n;
        vmin = vmin_n;
        vmax = vmax_n;
    }

    // 图像加工
    void processImage()
    {
        cvtColor(frame, frameHSV, COLOR_BGR2HSV);
        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);
        inRange(frameHSV, lower, upper, mask);
        GaussianBlur(mask, frameBlur, Size(1, 1), 0);
        dilate(frameBlur, frameDil, kernel);
    }

    // 找出轮廓
    void findcontours()
    {
        findContours(frameDil, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    }

    // 将灯条的上中下三点存储起来
    void Dengtiao()
    {
        for (int i = 0; i < contours.size(); i++)
        {
            int area = contourArea(contours[i]);
            if (area > 100)
            {
                RotatedRect rect = minAreaRect(contours[i]);
                Point2f vertices[4];
                rect.points(vertices);
                // 计算斜率
                if ((vertices[0].y - vertices[2].y) / (vertices[0].x - vertices[2].x) < -1 || (vertices[0].y - vertices[2].y) / (vertices[0].x - vertices[2].x) > 1)
                {
                    // for(int j =0 ;j<4;j++){
                    //     line(frame,vertices[j],vertices[(j+1)%4],Scalar(0,0,255),2);
                    // }
                    Point center_deng = Point((vertices[0].x + vertices[2].x) / 2, (vertices[0].y + vertices[2].y) / 2);
                    // circle(frame,center_deng,2,Scalar(0,255,0),2);
                    centerpoints.push_back(center_deng);
                    dengtiao.push_back(rect);
                    // 找出矩形的上下顶点
                    Point2f topLeft, topRight, downRight, downLeft, upcenter, downcenter;
                    float topY = numeric_limits<float>::max();
                    for (int j = 0; j < 4; j++)
                    {
                        if (vertices[j].y < topY)
                        {
                            topY = vertices[j].y;
                            topLeft = vertices[j];
                            downRight = vertices[(j + 2) % 4];
                        }
                    }
                    for (int j = 0; j < 4; j++)
                    {
                        if (vertices[j] != topLeft && abs(vertices[j].y - topY < 10))
                        {
                            topRight = vertices[j];
                            downLeft = vertices[(j + 2) % 4];
                            break;
                        }
                    }
                    // 上下中心点
                    upcenter = Point2f((topLeft.x + topRight.x) / 2, (topLeft.y + topRight.y) / 2);
                    downcenter = Point2f((downLeft.x + downRight.x) / 2, (downLeft.y + downRight.y) / 2);

                    toppoints.push_back(upcenter);
                    downpoints.push_back(downcenter);
                }
            }
        }
    }

    // 配对灯条，识别装甲板，并将装甲板的信息存储下来
    void Zhuangjiaban()
    {
        if (centerpoints.size() < 2 || dengtiao.size()<2)
        {
            cout << "未检测到足够灯条进行配对" << endl;
        }
        else{
            for (int i = 0; i < centerpoints.size() - 1; i++)
            {
                for (int j = i + 1; j < centerpoints.size(); j++)
                {
                    if (ispair(centerpoints[i], centerpoints[j],dengtiao[i],dengtiao[j]))
                    {
                        processPair(centerpoints[i], centerpoints[j], i, j);
                    }
                }
            }
        }
    }   

    // 判断灯条是否配对
    bool ispair(const Point2f &a, const Point2f &b,const RotatedRect &c,const RotatedRect &d)
    {
        if (abs(a.y- b.y) <20 &&
        abs((a.y- b.y) / (a.x - b.x)) < 0.3&&
        abs(a.x - b.x) <180 &&
        abs(c.angle-d.angle)<4
        )
    {
        return true;
    }
    else{
        return false;
    }
    }

    void processPair(const Point2f &a, const Point2f &b, int i, int j)
    {
        // 配对成功
        circle(frame, Point((a.x + b.x) / 2, (a.y + b.y) / 2), 2, Scalar(0, 255, 0), 2);
        Point2f center((a.x + b.x) / 2, (a.y + b.y) / 2);
        Size2f size((a.x - b.x) * 0.8, (a.x - b.x));
        float angleradians = atan2((a.y - b.y), (a.x - b.x));
        float angledegrees = angleradians * (180.0 / M_PI);
        RotatedRect rect2(center, size, angledegrees);
        Point2f vertices2[4];
        rect2.points(vertices2);
        zhuangjia_posion.push_back(rect2);
        // 框出装甲板矩形
        for (int k = 0; k < 4; k++)
        {
            line(frame, vertices2[k], vertices2[(k + 1) % 4], Scalar(255, 0, 0), 2);
        }

        vector<Point2f> zhuangjiaban;
        // 判断配对的两个灯条的左右顺序

        // 1、添加中间点
        if (a.x < b.x)
        {
            zhuangjiaban.push_back(a);
            zhuangjiaban.push_back(b);
        }
        else
        {
            zhuangjiaban.push_back(b);
            zhuangjiaban.push_back(a);
        }
        // 测试
        //  if (i >= toppoints.size() || j >= toppoints.size() || i >= downpoints.size() || j >= downpoints.size()) {
        //      cout << "索引越界:i=" << i << ", j=" << j << endl;
        //      return; // 退出方法
        //  }
        // 2、添加上边点
        if (toppoints[i].x < toppoints[j].x)
        {
            zhuangjiaban.push_back(toppoints[i]);
            zhuangjiaban.push_back(toppoints[j]);
        }
        else
        {
            zhuangjiaban.push_back(toppoints[j]);
            zhuangjiaban.push_back(toppoints[i]);
        }

        // 3、添加下边点
        if (downpoints[i].x < downpoints[j].x)
        {
            zhuangjiaban.push_back(downpoints[i]);
            zhuangjiaban.push_back(downpoints[j]);
        }
        else
        {
            zhuangjiaban.push_back(downpoints[j]);
            zhuangjiaban.push_back(downpoints[i]);
        }

        // 将配对的两个等条的六个点存放到一个数组中
        zhuangjiaban_sum.push_back(zhuangjiaban);
        _size = zhuangjiaban_sum.size();
        
    }
};

class KalmanFilter_1
{
public:
    KalmanFilter_1(double dt)
    {
        // 状态转移矩阵
        F = (Mat_<double>(4, 4) << 
            1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1);

        // 观测矩阵
        H = (Mat_<double>(2, 4) << 
            1, 0, 0, 0,
            0, 1, 0, 0);

        // 过程噪声协方差
        Q = Mat::eye(4, 4, CV_64F) * 10;

        // 测量噪声协方差
        R = Mat::eye(2, 2, CV_64F) * 1;

        // 初始状态和协方差
        reset();
        cout<<"重置：：：：：：：：：：：：：：：：：：：：：：："<<endl;
    }

    // 卡尔曼滤波步骤
    void update(const Mat &measurement)
    {
        // 1、预测
        X = F * X ; // 状态预测
        P = F * P * F.t() + Q;   // 协方差预测
        // 2、更新
        Mat K = P * H.t() * (H * P * H.t() + R).inv(); // 卡尔曼增益
        // cout<<"K"<<K<<endl;
        X = X + K * (measurement - H * X); // 更新状态
        P = (Mat::eye(4, 4, CV_64F) - K * H) * P;

        cout << "调用成功" << endl;
    }

    // 重置卡尔曼滤波器
    void reset()
    {
        X = Mat::zeros(4, 1, CV_64F); // 初始状态：x,y,vx,vy
        P = Mat::eye(4, 4, CV_64F); // 初始协方差
    }


    Mat get_X(){
        return X;
    }

    void set_dt(double input_dt){
        dt=input_dt;
    }

    void set_v(double vx,double vy){
        X.at<double>(2)=vx;
        X.at<double>(3)=vy;
    }

private:
    Mat F, B, H, Q, R, X, P;
    double dt;
};

class Armor
{
public:
    Armor(Mat input_image) : image(input_image), distance(0)
    {
        if (input_image.empty())
        {
            cout << "输入图像为空" << endl;
            return;
        }
    }

    // 进行数字识别，姿态解算
    void classify(const RotatedRect rect,const vector<Point2f> zitai)
    {
        if(!zitai.empty()){
            cout<<rect.size.width<<"width"<<endl;
            cout<<rect.size.height<<"height"<<endl;
            Number(rect);
            Zitai(zitai);    
        }

    }

    int get_type()
    {
        return type;
    }

    double get_distance()
    {
        return distance;
    }

    Point2f get_position()
    {
        return position;
    }

    // 用明显颜色框出目标装甲板矩形
    void draw_target()
    {
        for (int i = 0; i < 4; i++)
        {
            line(image, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 255), 2);
        }

        position = (vertices[0] + vertices[2]) / 2;
        putText(image, "Target", Point(position.x - 20, position.y - 100), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2);
    }

    

    Mat Image, image, Imagegray, Imagebinary, rvec, tvec;
    Point2f vertices[4];
    Point2f position;
    Point2f previous_center;
    Point2f previous_velocity;
    double previous_time;
    vector<RotatedRect> dengtiao_num;
    vector<int> dengtiaoban_type;

private:
    int type;
    double distance, x, y, z;
    string MODLE_PATH = "/home/l/Desktop/opencv/cnn_model_1.onnx";

    // 识别数字
    void Number(const RotatedRect rect)
    {
        // 提取装甲板

        rect.points(vertices);
    //     Rect bounding_rect =boundingRect(Mat(vector<Point>(vertices,vertices + 4)));
    //     bounding_rect &=Rect(0,0,image.cols,image.rows);
    //     Mat cropped_frame = image(bounding_rect);

    //     if(cropped_frame.empty()){
    //         cout<<"裁剪后图像为空，无法进行数字识别"<<endl;
    //         return;
    //     }

    //     // 识别数字
    //     ov::Core core;
    //     ov::CompiledModel complied_model = core.compile_model(MODLE_PATH,"CPU");
    //     ov::InferRequest infer_request = complied_model.create_infer_request();
    //     ov::Tensor input_tensor = infer_request.get_input_tensor(0);
    //     ov::Shape tensor_shape = input_tensor.get_shape();

    //     size_t channel = tensor_shape[1];
    //     size_t height = tensor_shape[2];
    //     size_t width = tensor_shape[3];

    //     //数字图像处理
    //     cvtColor(cropped_frame,Imagegray,COLOR_BGR2GRAY);
    //     cv::resize(Imagegray,Image,cv::Size(width,height));
        

    //     // cvtColor(Image,Imagegray,COLOR_BGR2GRAY);
    //     double value = 50;
    //     threshold(Imagegray,Imagebinary,value,255,THRESH_BINARY); //二值化
    //     Imagebinary.convertTo(Image,CV_32F,1.0/255.0);   //归一化
    //     resize(Image,Image,Size(200,200));
    //     resize(Imagegray,Imagegray,Size(200,200));
    //     resize(Imagebinary,Imagebinary,Size(200,200));
    //     imshow("Image",Image);
    //     imshow("Imagegray",Imagegray);
    //     imshow("Imagebinary",Imagebinary);



    //     float* image_data = input_tensor.data<float>();

    //     for(size_t c=0;c<channel;c++){
    //         for (size_t h = 0; h < height; h++){
    //             for (size_t w = 0; w < width; w++){
    //                 size_t index= c*width*height+h*width+w;
    //                 image_data[index] = Image.at<cv::Vec3f>(h,w)[c];
    //             }
    //         }
    //     }
    //     infer_request.infer();

    //     auto output = infer_request.get_output_tensor();

    //     const float* output_num = output.data<const float>();
    //     int number = std::max_element(output_num,output_num+5) -output_num+1;


    //     int text_x = bounding_rect.x;
    //     int text_y = bounding_rect.y + bounding_rect.height;
    //     text_y = std::max(text_y - 10, 0);

    //     // 识别数字并在图像上显示
    //     string text = to_string(number);
    //     putText(image, text, Point(text_x, text_y), FONT_HERSHEY_COMPLEX, 2, Scalar(0, 255, 0), 1);
    //     dengtiaoban_type.push_back(number);
    //     // 将识别到的装甲板分类
    //     type = number;
    }

    // 解算装甲板姿态
    void Zitai(const vector<Point2f> zitai)
    {
        // 相机参数与畸变函数
        Mat cameraMatrix = (Mat_<double>(3, 3) << 623.5383, 0., 640, 0., 1108.513, 360, 0., 0., 1.);
        Mat discoeffs = Mat::zeros(5,1,CV_64F);
        vector<Point3f> objectpoints = {
            {-67, 0, 0},     // 左中
            {67, 0, 0},      // 右中
            {-67, 27.5, 0},  // 左上
            {67, 27.5, 0},   // 右上
            {-67, -27.5, 0}, // 左下
            {67, -27.5, 0},  // 右下
        };

        vector<Point2f> imagepoints;
        for (int i = 0; i < 6; i++)
        {
            imagepoints.push_back(zitai[i]);
        }

        // 计算姿态
        solvePnP(objectpoints, imagepoints, cameraMatrix, discoeffs, rvec, tvec);

        // 将旋转向量转化为旋转矩形
        Mat R;
        Rodrigues(rvec, R);

        Mat P_world = (Mat_<double>(3, 1) << 0, 0, 0);
        Mat P_camera = R * P_world + tvec;

        // 计算距离
        distance = norm(P_camera);
    }
};

class AutoShoot : public rclcpp::Node
{
public:
    AutoShoot()
        : Node("auto_shoot_node"),kf(0.02),previous_position(0,0),current_position(0,0)
    {
        // 订阅相机图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_image", 10,
            std::bind(&AutoShoot::imageCallback, this, std::placeholders::_1));

        // 订阅云台角度
        receive_data_sub_ = this->create_subscription<tdt_interface::msg::ReceiveData>(
            "/real_angles", 10,
            std::bind(&AutoShoot::receiveCallback, this, std::placeholders::_1));

        // 订阅栅格地图
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&AutoShoot::mapCallback, this, std::placeholders::_1));

        // 订阅当前机器人位姿
        position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/position", 10,
            std::bind(&AutoShoot::positionCallback, this, std::placeholders::_1));

        // 订阅当前真实速度
        real_speed_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/real_speed", 10,
            std::bind(&AutoShoot::realSpeedCallback, this, std::placeholders::_1));

        // 订阅目标点位姿
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&AutoShoot::goalPoseCallback, this, std::placeholders::_1));

        // 发布目标云台角度
        send_data_pub_ = this->create_publisher<tdt_interface::msg::SendData>("/target_angles", 10);

        // 发布目标速度
        speed_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/target_speed", 10);

        // 发布比赛开始信号
        game_start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/game_start", 10);

        publishGameStartSignal();
    }

private:
    void publishGameStartSignal()
    {
        auto msg = std::make_shared<std_msgs::msg::Bool>();
        msg->data = true;
        game_start_pub_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Game start");
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        std::vector<uint8_t> jpeg_data(msg->data.begin(), msg->data.end());
        frame = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode image.");
            return;
        }
        /********************处理你的图像*********************/

        vector<Armor> armors;
        Detector detector(frame);
        detector.select('b');
        detector.detect();
        cout<<"1111111111111"<<endl;
        frame=detector.Imshow_frame();
        cout<<"2222222222222"<<endl;
        
        Mat dil=detector.Imshow_Dil();
        
        //如果检测到装甲板
        if(detector.get_zhuangjia_num()>0){
            for(int i=0;i<detector.get_zhuangjia_num();i++){
                Armor newarmor(frame);
                cout<<"rrrrrrrrrrrrrrrrrrrrr"<<endl;
                cout<<detector.get_zhuangjia_position().size()<<"          77777777777777777777777777777"<<endl;
                cout<<detector.get_zhuangjia_passage().size()<<"       66666666666666666666666666666666"<<endl;
                cout<<detector.get_zhuangjia_num()<<"      88888888888888888888"<<endl;
                newarmor.classify(detector.get_zhuangjia_position()[i],detector.get_zhuangjia_passage()[i]);
                cout<<detector.get_zhuangjia_position().size()<<"77777777777777777777777777777"<<endl;
                cout<<detector.get_zhuangjia_passage().size()<<"66666666666666666666666666666666"<<endl;

                armors.push_back(newarmor);
                number_1=0;   //number_1控制转头
                number_2=number_2+1;  //number_2控制开火
                iszhuangjia=true;
            }
        }
        else{
            cout<<"没有检测到装甲板"<<endl;
            iszhuangjia=false;
            number_1=number_1+1;
            if(qingkong){
                number_2=0;
                qingkong=false;
            }
            if(number_1>90){
                pitch=0;
                yaw+=5;
            }
            if(number_1>20){
                qingkong=true;
            }
        }

        //判断是否开火
         if(number_2>5){
            iffire=true;
        }
        else{
            iffire=false;
        }
        
        //寻找距离最短的装甲板
        if(!armors.empty()){
            double shortest_dis=armors[0].get_distance();
            int num =0;
        
            for(int i=1; i<armors.size();i++)
            {
                if(armors[i].get_distance()<shortest_dis){
                    shortest_dis=armors[i].get_distance();
                    num=i;
                }
            }
            distance=shortest_dis;
            cout<<"距离为  "<<shortest_dis<<endl;
            cout<<"lllllllllllllllllllllllllllllllllllllllll"<<endl;
            //框出目标
            armors[num].draw_target();
            current_position =armors[num].get_position();
        
            Mat measurement =(Mat_<double>(2,1)<<current_position.x,current_position.y);
            kf.update(measurement);
            //计算子弹飞行时间
            time=shortest_dis/2800;
            Pred_x=kf.get_X().at<double>(0) + kf.get_X().at<double>(2) * time;
            Pred_y=kf.get_X().at<double>(1) + kf.get_X().at<double>(3) * time;
            Point2d Pred(Pred_x,Pred_y);
            cout<<current_position<<"现在"<<endl;
            cout<<previous_position<<"先前"<<endl;
            cout<<Pred<<"zzzzzzzzzzzzzzzzzzzzzzz"<<endl;
            circle(frame,Pred,5,Scalar(0,0,255),-1);
            //更新位置
            previous_position = current_position;

            // jiaodu_x=atan(armors[num].tvec.at<double>(0)/armors[num].tvec.at<double>(3));
            // jiaodu_y=atan(armors[num].tvec.at<double>(1)/armors[num].tvec.at<double>(3));

            // cout<<armors[num].tvec.at<double>(0)<<"::::::x::::: "<<endl;
            // cout<<armors[num].tvec.at<double>(1)<<"::::::y::::: "<<endl;
            // cout<<armors[num].tvec.at<double>(2)<<"::::::z::::: "<<endl;

        }
     











        /******************************************/
        cv::imshow("Camera Image", frame);
        cv::imshow("dil",dil);
        cv::waitKey(1);


        /********************发布你应该发布的角度**************************/
        if(!armors.empty()){
            double threshold = 1;
            if(Pred_x<frame.cols/2 && abs(frame.cols/2-Pred_x)>threshold){
                if((frame.cols/2-Pred_x)*(90.0/frame.cols)>5){
                    yaw=yaw-5;
                }
                else{
                yaw=yaw-(frame.cols/2-Pred_x)*(90.0/frame.cols);
                }
                cout<<"左转"<<endl;
            }
            if(Pred_x>frame.cols/2 && abs(Pred_x-frame.cols/2)>threshold){
                if((Pred_x-frame.cols/2)*(90.0/frame.cols)>5){
                    yaw=yaw+5;
                }
                else{
                yaw=yaw+(Pred_x-frame.cols/2)*(90.0/frame.cols);
                }
                cout<<"右转"<<endl;
            }
            if(Pred_y<frame.rows/2 && abs(frame.rows/2-Pred_y)>threshold){
                if((frame.rows/2-Pred_y)*(60.0/frame.rows)>1){
                    pitch=pitch+1;
                }
                else{
                    pitch=pitch+(frame.rows/2-Pred_y)*(60.0/frame.rows);
                }
                cout<<"上转"<<endl;
            }
            if(Pred_y>frame.rows/2 && abs(Pred_y-frame.rows/2)>threshold){
                if((Pred_y-frame.rows/2)*(90.0/frame.rows)>1){
                    pitch=pitch-1;
                }
                else{
                    pitch=pitch-(Pred_y-frame.rows/2)*(90.0/frame.rows);
                }
                cout<<"下转"<<endl;
            }


            // if(current_position.x<frame.cols/2){
            //     yaw_1=yaw_1-(frame.cols/2-Pred_x)*(90.0/frame.cols);
            //     cout<<(90/frame.cols)<<"qqqqqqqqqqqqqqqqqqq"<<endl;
            //     cout<<"左转"<<endl;
            // }
            // if(current_position.x>frame.cols/2){
            //     yaw_1=yaw_1+(Pred_x-frame.cols/2)*(90.0/frame.cols);
            //     cout<<"右转"<<endl;
            // }
            // if(current_position.y<frame.rows/2){
            //     pitch=pitch+(frame.rows/2-Pred_y)*(60.0/frame.rows);
            //     cout<<"上转"<<endl;
            // }
            // if(current_position.y>frame.rows/2){
            //     pitch=pitch-(Pred_y-frame.rows/2)*(90.0/frame.rows);
            //     cout<<"下转"<<endl;
            // }
        
            // if(Pred_x<frame.cols/2 && abs(frame.cols/2-Pred_x)>threshold){
            //     yaw=yaw+jiaodu_x;
            //     cout<<jiaodu_x<<" :::::::::: "<<jiaodu_y<<"qqqqqqqqqqqqqqqqqqq"<<endl;
            //     cout<<"左转"<<endl;
            // }
            // if(Pred_x>frame.cols/2 && abs(Pred_x-frame.cols/2)>threshold){
            //     yaw=yaw+jiaodu_x;
            //     cout<<jiaodu_x<<" :::::::::: "<<jiaodu_y<<"qqqqqqqqqqqqqqqqqqq"<<endl;
            //     cout<<"右转"<<endl;
            // }
            // if(Pred_y<frame.rows/2 && abs(frame.rows/2-Pred_y)>threshold){
            //     pitch=pitch-jiaodu_y;
            //     cout<<jiaodu_x<<" :::::::::: "<<jiaodu_y<<"qqqqqqqqqqqqqqqqqqq"<<endl;
            //     cout<<"上转"<<endl;
            // }
            // if(Pred_y>frame.rows/2 && abs(Pred_y-frame.rows/2)>threshold){
            //     pitch=pitch-jiaodu_y;
            //     cout<<jiaodu_x<<" :::::::::: "<<jiaodu_y<<"qqqqqqqqqqqqqqqqqqq"<<endl;
            //     cout<<"下转"<<endl;
            // }

        }
        
        // yaw_1=yaw;
        // if(yaw_1>360){yaw_1=fmod(yaw_1,360);}
        // if(!armors.empty()){
        //      //跟随
        //     if(distance>1000){
        //         if (yaw_1=0){
        //             speed_x=0;
        //             speed_y=2;
        //         }
                
        //         if(yaw_1<90 && yaw_1>0){
        //             speed_x=2;
        //             speed_y=2;
        //         }
        //         if(yaw_1=90){
        //             speed_x=2;
        //             speed_y=0;
        //         }
        //         if(yaw_1>90 &&yaw_1<180){
        //             speed_x=2;
        //             speed_y=-2;
        //         }
        //         if(yaw_1=180){
        //             speed_y=-2;
        //             speed_x=0;
        //         }
        //         if(yaw_1>180 &&yaw_1<270){
        //             speed_x=-2;
        //             speed_y=-2;
        //         }
        //         if(yaw_1=270){
        //             speed_x=-2;
        //             speed_y=0;
        //         }
        //         if (yaw_1>270 && yaw_1<360){
        //             speed_x=-2;
        //             speed_y=2;
        //         }
                
        //     }
        //     else{
        //         speed_x=0;
        //         speed_y=0;
        //     }
        // }
        cout<<"pitch : "<<pitch<<"yaw : "<<yaw<<"yaw_1 : "<<yaw_1<<"uuuuuuuuuuuuuuuuuuuuuuuuuuuu"<<endl;
        auto send_data_msg = std::make_shared<tdt_interface::msg::SendData>();
        send_data_msg->pitch = pitch;
        send_data_msg->yaw = yaw;
        send_data_msg->if_shoot = iffire;

        send_data_pub_->publish(*send_data_msg);
        cout<<"pitch : "<<pitch<<"yaw : "<<yaw<<"uuuuuuuuuuuuuuuuuuuuuuuuuuuu"<<endl;
        
    }

    void realSpeedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {

        float real_linear_speed_x = msg->twist.linear.x;
        float real_linear_speed_y = msg->twist.linear.y;
        /****************处理回调速度************************/
        RCLCPP_DEBUG( this->get_logger(), "Real linear speed: [x: %f, y: %f]", real_linear_speed_x, real_linear_speed_y);





        /*******************发布期望速度***********************/
        float max_speed_x;
        float max_speed_y;
            if(!path_node.empty()){
                //每到达一个拐点便删除
                if(abs(Begin_x-path_node[0].x*25)<25 && abs(Begin_y-path_node[0].y*25)<25){
                    path_node.erase(path_node.begin());
                    max_speed_x=0;
                    max_speed_y=0;
                }
                //一档
                if(abs(Begin_x-path_node[0].x*25)<75){
                    max_speed_x=1.5;
                }
                //二档
                if(abs(Begin_x - path_node[0].x * 25) > 75 && abs(Begin_x - path_node[0].x * 25) <= 150) {
                    max_speed_x=3;
                }
                //三档
                if(abs(Begin_x-path_node[0].x*25)>150){
                    max_speed_x=5;
                }

                
                if(abs(Begin_y-path_node[0].y*25)<75){
                    max_speed_y=1;
                }
                if(abs(Begin_y - path_node[0].y * 25) > 75 && abs(Begin_y - path_node[0].y * 25) <= 150) {
                    max_speed_y=2;
                }
                if(abs(Begin_y-path_node[0].y*25)>150){
                    max_speed_y=3;
                }

                //移动
                //往右
                if(Begin_x-path_node[0].x*25<0){
                    speed_x=max_speed_x;
                }
                //往左
                if(Begin_x-path_node[0].x*25>0){
                    speed_x=-max_speed_x;
                }
                //往下
                if(Begin_y-path_node[0].y*25<0){
                    speed_y=max_speed_y;
                }
                //往上
                if(Begin_y-path_node[0].y*25>0){
                    speed_y=-max_speed_y;
                }
                cout<<"下一个目标点为"<<"("<<path_node[0].x<<","<<path_node[0].y<<")"<<"::::::::::::::::::::::::::::"<<endl;
            }
            else{
                path_node.clear();
                speed_x=0;
                speed_y=0;
            }
     
        
        cout<<speed_x<<","<<speed_y<<"::::::::::::速度::::::::::::::::::::::::::::::速度:::::::::::::::::::::::::::::::速度"<<endl;

        auto target_speed_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        target_speed_msg->header.stamp = this->get_clock()->now();
        target_speed_msg->twist.linear.x=speed_x;
        target_speed_msg->twist.linear.y=speed_y;
        speed_pub_->publish(*target_speed_msg);
    }

    // 云台角度回调
    void receiveCallback(const tdt_interface::msg::ReceiveData::SharedPtr msg)
    {
        pitch = msg->pitch;
        yaw = msg->yaw;
    }

    // 栅格地图回调
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        /*****************************保存或处理你的地图*****************************/
        int width = msg->info.width;
        int height = msg->info.height;
        cv::Mat map_image(height, width, CV_8UC1);

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int index = x + y * width;
                int8_t occupancy_value = msg->data[index];
                uint8_t pixel_value = 0;

                if (occupancy_value == 0)
                    pixel_value = 255;
                else if (occupancy_value == 100)
                    pixel_value = 0;
                else
                    pixel_value = 128;

                map_image.at<uint8_t>(y, x) = pixel_value;
            }
        }

        cout<<"接收到地图"<<endl;
        map=map_image;
        cout<<"保存地图"<<endl;


        cv::imshow("Occupancy Grid Map", map_image);
        cv::waitKey(1);
    }

    void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
       /***********************处理自身位置信息**************************/
        beginP.x=static_cast<int>(round(msg->pose.position.x));
        Begin_x=msg->pose.position.x*25;
        beginP.y=static_cast<int>(round(msg->pose.position.y));
        Begin_y=msg->pose.position.y*25;
        RCLCPP_INFO(this->get_logger(), "Robot position: [x: %f, y: %f, z: %f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        /***********************处理目标位置信息**************************/
        endP.x=static_cast<int>(round(msg->pose.position.x));
        endP.y=static_cast<int>(round(msg->pose.position.y));

        path_node.clear();
        speed_x=0;
        speed_y=0;
        gd.guide(map,beginP,endP);
        if(gd.qidong){
            edit_map=gd.get_edit_map();
            resize(edit_map,edit_map,Size(1250,1250),0,0,cv::INTER_LINEAR);
            Mat map_color;
            cvtColor(map,map_color,cv::COLOR_GRAY2BGR);
            gd.drawPath(map_color);
            gd.drawNode(map_color,path_node);
            cv::imshow("map1",map_color);
            cv::imshow("map2",edit_map);
            cout<<"zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz"<<endl;
            cout<<endP.x<<"       "<<endP.y<<endl;
        }
        RCLCPP_INFO(this->get_logger(), "Goal position received: [x: %f, y: %f, z: %f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

   

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr real_speed_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr game_start_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<tdt_interface::msg::SendData>::SharedPtr send_data_pub_;
    rclcpp::Subscription<tdt_interface::msg::ReceiveData>::SharedPtr receive_data_sub_;
    float yaw = 0;
    float pitch = 0;
    KalmanFilter_1 kf;
    Point2f previous_position;
    Point2f current_position;
    double dt=0.02;
    double time;
    double Pred_x;
    double Pred_y;
    int number_1;
    int number_2;
    bool qingkong;
    bool iffire;
    Guide gd;
    Guide::MyPoint beginP;
    Guide::MyPoint endP;
    Mat map;
    Mat edit_map;
    double distance;
    vector<Guide::MyPoint> path_node;
    float speed_x;
    float speed_y;
    float Begin_x;
    float Begin_y;
    float yaw_1;
    bool iszhuangjia;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoShoot>());
    rclcpp::shutdown();
    return 0;
}

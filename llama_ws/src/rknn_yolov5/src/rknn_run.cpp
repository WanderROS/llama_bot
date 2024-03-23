#include "rknn_run.h"
#include "postprocess.h"
#include "preprocess.h"

#define printf ROS_INFO

// 输入形式：话题(配合其他摄像头、RTSP等采集节点使用)，或图片文件夹(离线测试)
// 输出形式：话题，或图片文件（图片文件用于测试）
RknnRun::RknnRun() : nh("~")
{
    nh.param<std::string>("model_file", model_file, ""); //$(find rknn_yolo)/config/xxx.rknn
    nh.param<std::string>("yaml_file", yaml_file, "");
    nh.param<std::string>("offline_images_path", offline_images_path, "");
    nh.param<std::string>("offline_output_path", offline_output_path, "");

    nh.param<std::string>("sub_image_topic", sub_image_topic, "/camera/image_raw");
    nh.param<std::string>("pub_image_topic", pub_image_topic, "/camera/image_det");

    nh.param<bool>("is_offline_image_mode", is_offline_image_mode, false); // 离线用于图片测试

    nh.param<bool>("print_perf_detail", print_perf_detail, false);
    nh.param<bool>("use_multi_npu_core", use_multi_npu_core, false);
    nh.param<bool>("output_want_float", output_want_float, false);

    nh.param<double>("conf_threshold", conf_threshold, 0.25);
    nh.param<double>("nms_threshold", nms_threshold, 0.45);

    // 读取classes配置
    cv::FileStorage fs(yaml_file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        ROS_WARN("Failed to open file %s\n", yaml_file.c_str());
        ros::shutdown();
        return;
    }

    fs["nc"] >> cls_num;
    fs["label_names"] >> label_names;

    printf("cls_num (nc) =%d label_names len=%d\n", cls_num, label_names.size());

    image_pub = nh.advertise<sensor_msgs::Image>(pub_image_topic, 10);

    image_sub = nh.subscribe(sub_image_topic, 10, &RknnRun::sub_image_callback, this);
}

void RknnRun::sub_image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    // ROS_WARN_STREAM("frame_id="<<msg->header.frame_id);
    capture_data_queue.push(msg);
}

void sig_handler(int sig)
{
    if (sig == SIGINT)
    {
        printf("Ctrl C pressed, shutdown\n");
        ros::shutdown();
        // exit(0);//在ROS里不要调用exit(),会卡住
    }
}

static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz)
{
    unsigned char *data;
    int ret;

    data = NULL;

    if (NULL == fp)
    {
        return NULL;
    }

    ret = fseek(fp, ofst, SEEK_SET);
    if (ret != 0)
    {
        printf("blob seek failure.\n");
        return NULL;
    }

    data = (unsigned char *)malloc(sz);
    if (data == NULL)
    {
        printf("buffer malloc failure.\n");
        return NULL;
    }
    ret = fread(data, 1, sz, fp);
    return data;
}

static unsigned char *load_model(const char *filename, int *model_size)
{
    FILE *fp;
    unsigned char *data;

    fp = fopen(filename, "rb");
    if (NULL == fp)
    {
        printf("Open file %s failed.\n", filename);
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);

    data = load_data(fp, 0, size);

    fclose(fp);

    *model_size = size;
    return data;
}

static void dump_tensor_attr(rknn_tensor_attr *attr)
{
    std::string shape_str = attr->n_dims < 1 ? "" : std::to_string(attr->dims[0]);
    for (int i = 1; i < attr->n_dims; ++i)
    {
        shape_str += ", " + std::to_string(attr->dims[i]);
    }

    printf("  index=%d, name=%s, n_dims=%d, dims=[%s], n_elems=%d, size=%d, w_stride = %d, size_with_stride=%d, fmt=%s, "
           "type=%s, qnt_type=%s, "
           "zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, shape_str.c_str(), attr->n_elems, attr->size, attr->w_stride,
           attr->size_with_stride, get_format_string(attr->fmt), get_type_string(attr->type),
           get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}
double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

int RknnRun::run_infer_thread()
{
    signal(SIGINT, sig_handler); // SIGINT 信号由 InterruptKey 产生，通常是 CTRL +C 或者 DELETE

    int img_width;
    int img_height;
    int ret;

    InferData infer_data;
    // init rga context
    rga_buffer_t src;
    rga_buffer_t dst;
    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));

    struct timeval start_time, stop_time;
    unsigned char *model_data = nullptr;
    int model_data_size = 0;

    printf("Loading model ...\n");
    if (access(model_file.c_str(), 0) != 0) // 模型文件不存在，则退出
    {
        ROS_WARN("%s model file is not exist!!!", model_file.c_str());
        ros::shutdown();
    }
    model_data = load_model(model_file.c_str(), &model_data_size);
    ret = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
    if (ret < 0)
    {
        printf("rknn_init error,model load err! ret=%d\n", ret);
        ros::shutdown();
    }
    rknn_sdk_version version;
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
    if (ret < 0)
    {
        printf("rknn_init error sdk version ret=%d\n", ret);
        ros::shutdown();
    }
    printf("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0)
    {
        printf("rknn_init error ret=%d\n", ret);
        ros::shutdown();
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);
    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret < 0)
        {
            printf("rknn_init error ret=%d\n", ret);
            ros::shutdown();
        }
        dump_tensor_attr(&(input_attrs[i]));
    }
    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        dump_tensor_attr(&(output_attrs[i]));
    }

    for (int i = 0; i < io_num.n_output; ++i)
    {
        out_scales.push_back(output_attrs[i].scale);
        out_zps.push_back(output_attrs[i].zp);
    }

    int channel = 3;
    int width = 0;
    int height = 0;
    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        printf("model is NCHW input fmt\n");
        channel = input_attrs[0].dims[1];
        height = input_attrs[0].dims[2];
        width = input_attrs[0].dims[3];
    }
    else
    {
        printf("model is NHWC input fmt\n");
        height = input_attrs[0].dims[1];
        width = input_attrs[0].dims[2];
        channel = input_attrs[0].dims[3];
    }

    printf("model input height=%d, width=%d, channel=%d\n", height, width, channel);

    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = width * height * channel;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].pass_through = 0;

    // 加载图片
    cv::Size target_size(width, height);
    cv::Mat resized_img(target_size.height, target_size.width, CV_8UC3);
    std::vector<cv::String> image_files;
    int image_id = 0;
    if (is_offline_image_mode)
    {

        cv::glob(offline_images_path, image_files, false); // 三个参数分别为要遍历的文件夹地址；结果的存储引用；是否递归查找，默认为false
        if (image_files.size() == 0)
        {
            ROS_WARN_STREAM("offline_images_path read image files!!! : " << offline_images_path);
            ros::shutdown();
        }
        else
        {
            for (int i = 0; i < image_files.size(); i++)
            {
                ROS_INFO_STREAM("offline_images: " << image_files[i]);
            }
        }

        if (access(offline_output_path.c_str(), 0) != 0)
        {
            // if this folder not exist, create a new one.
            if (mkdir(offline_output_path.c_str(), 0777) != 0)
            {
                ROS_INFO_STREAM("offline_output_path mkdir fail!!! : " << offline_output_path);
                ros::shutdown();
            }
        }
    }
    ROS_INFO("rknn init finished");

    while (1)
    {
        if (is_offline_image_mode)
        {
            if (image_id >= image_files.size())
            {
                printf("image read finished\n");
                return 0;
            }
            infer_data.orig_img = cv::imread(image_files[image_id]);
            cv::cvtColor(infer_data.orig_img, infer_data.orig_img, cv::COLOR_BGR2RGB); // 转为RGB用于推理

            infer_data.header.seq = image_id;
            infer_data.header.stamp = ros::Time::now(); // 离线图片时间戳
            infer_data.header.frame_id = "image";

            image_id++;

            usleep(30 * 1000); // 30ms
        }
        else
        {
            sensor_msgs::ImageConstPtr msg;
            capture_data_queue.wait_and_pop(msg);

            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "rgb8"); // ROS消息转OPENCV
            if (cv_ptr->image.empty())
            {
                ROS_WARN("cv_ptr->image.empty() !!!");
                continue;
            }

            infer_data.orig_img = cv_ptr->image.clone(); // 接收的ROS消息已经是RGB格式了，不需要再转换为RGB用于推理
            infer_data.header = msg->header;
        }
        img_width = infer_data.orig_img.cols;
        img_height = infer_data.orig_img.rows;

        infer_data.mod_size = cv::Size(width, height);
        infer_data.img_size = infer_data.orig_img.size();
        // 缩放
        if (img_width != width || img_height != height)
        {
            printf("imagesize: img_width: %d img_height:%d,width:%d,height:%d\n", img_width, img_height, width, height);
            printf("resize image by rga\n");
            ret = resize_rga(src, dst, infer_data.orig_img, resized_img, infer_data.mod_size);
            if (ret != 0)
            {
                printf("resize image by rga error\n");
                ros::shutdown();
            }
            cv::imwrite("resize_input.jpg", resized_img);
            inputs[0].buf = (void *)resized_img.data;
        }
        else
        {
            inputs[0].buf = (void *)infer_data.orig_img.data;
        }

        gettimeofday(&start_time, NULL);
        rknn_inputs_set(ctx, io_num.n_input, inputs);

        memset(infer_data.outputs, 0, sizeof(infer_data.outputs));
        for (int i = 0; i < io_num.n_output; i++)
        {
            infer_data.outputs[i].want_float = 0;
        }
        // 执行推理
        ret = rknn_run(ctx, NULL);
        if (print_perf_detail) // 是否打印每层运行时间
        {
            rknn_perf_detail perf_detail;
            ret = rknn_query(ctx, RKNN_QUERY_PERF_DETAIL, &perf_detail, sizeof(perf_detail));
            printf("perf_detail: %s\n", perf_detail.perf_data);
        }
        // 读取结果
        ret = rknn_outputs_get(ctx, io_num.n_output, infer_data.outputs, NULL);
        gettimeofday(&stop_time, NULL);
        printf("once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);
        process_data_queue.push(infer_data);
    }

    ret = rknn_destroy(ctx);

    if (model_data)
    {
        free(model_data);
    }
}

int RknnRun::run_process_thread()
{
    signal(SIGINT, sig_handler); // SIGINT 信号由 InterruptKey 产生，通常是 CTRL +C 或者 DELETE
    printf("post process config: conf_threshold = %.2f, nms_threshold = %.2f\n",
           conf_threshold, nms_threshold);

    int ret;
    char text[256];
    unsigned int frame_cnt = 0;
    double t = 0, last_t = 0;
    double fps;
    struct timeval tv;
    int channel = 3;
    int width = 0;
    int height = 0;
    cv::Scalar color_list[12] = {
        cv::Scalar(0, 0, 255),
        cv::Scalar(0, 255, 0),
        cv::Scalar(255, 0, 0),

        cv::Scalar(0, 255, 255),
        cv::Scalar(255, 0, 255),
        cv::Scalar(255, 255, 0),

        cv::Scalar(0, 128, 255),
        cv::Scalar(0, 255, 128),

        cv::Scalar(128, 0, 128),
        cv::Scalar(255, 0, 128),

        cv::Scalar(128, 255, 0),
        cv::Scalar(255, 128, 0)};

    sensor_msgs::ImagePtr image_msg;

    BOX_RECT pads;
    memset(&pads, 0, sizeof(BOX_RECT));
    while (1)
    {
        InferData infer_data;
        process_data_queue.wait_and_pop(infer_data);
        printf("received images!!!!!!\n");
        cv::Mat infer_data_orig_img = infer_data.orig_img.clone(); // 拷贝一份绘制，否则会影响到原始图片data
        rknn_output *outputs = infer_data.outputs;                 // 获取结构体的指针
        cv::Size mod_size = infer_data.mod_size;
        cv::Size img_size = infer_data.img_size;

        float scale_w = (float)img_size.width / mod_size.width;
        float scale_h = (float)img_size.height / mod_size.height;

        std::vector<int> out_index = {0, 1, 2};
        detect_result_group_t detect_result_group;
        printf("post_process model:%d,%d,scale:%f,%f!!!!!!!!!\n", mod_size.height, mod_size.width, scale_w, scale_h);

        post_process((int8_t *)outputs[0].buf, (int8_t *)outputs[1].buf, (int8_t *)outputs[2].buf, mod_size.height, mod_size.width,
                     conf_threshold, nms_threshold, pads, scale_w, scale_h, out_zps, out_scales, &detect_result_group,label_names);

        // 画框和概率
        char text[256];
        for (int i = 0; i < detect_result_group.count; i++)
        {
            detect_result_t *det_result = &(detect_result_group.results[i]);
            sprintf(text, "%s %.1f%%", det_result->name, det_result->prop * 100);
            printf("%s @ (%d %d %d %d) %f\n", det_result->name, det_result->box.left, det_result->box.top,
                   det_result->box.right, det_result->box.bottom, det_result->prop);
            int x1 = det_result->box.left;
            int y1 = det_result->box.top;
            int x2 = det_result->box.right;
            int y2 = det_result->box.bottom;
            rectangle(infer_data_orig_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(256, 0, 0, 256), 3);
            putText(infer_data_orig_img, text, cv::Point(x1, y1 + 12), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
        }

        frame_cnt++;
        if (frame_cnt % 30 == 0)
        {
            gettimeofday(&tv, NULL);
            t = tv.tv_sec + tv.tv_usec / 1000000.0;

            fps = 30.0 / (t - last_t);
            // ROS_WARN("det publish_fps=%.1f (%.1f ms)",fps,1000.0/fps);
            last_t = t;
        }
        image_msg = cv_bridge::CvImage(infer_data.header, "rgb8", infer_data_orig_img).toImageMsg(); // opencv-->ros
        image_pub.publish(image_msg);                                                                // 发布绘制了检测框的图像

        if (is_offline_image_mode) // 离线模式会保存图片
        {
            std::string output_file = offline_output_path + "/" + std::to_string(infer_data.header.seq) + ".jpg";
            cv::imwrite(output_file, infer_data_orig_img);

            ROS_INFO_STREAM("saved: " << output_file);
        }
    }
}
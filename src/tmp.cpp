//
// Created by shawn-sony-u16 on 18-11-25.
//

//    FLAGS_logbufsecs =1;

//    struct msg_update_type {
//        union {
//            struct {
//                uint8_t path: 1;
//                uint8_t speed: 1;
//                uint8_t speed1: 1;
//                uint8_t speed2: 1;
//                uint8_t speed3: 1;
//                uint8_t speed4: 1;
//                uint8_t speed5: 1;
//                uint8_t speed6: 1;
//            };
//            uint8_t result;
//        } update;
//        uint8_t yes;
//        struct {
//            bool collision;
//            bool ramp;
//        } addition;
//    }msg_update;
//    msg_update.update.result = 255;
//    msg_update.update.path = 0;
//    msg_update.update.items.speed = 0;
//    msg_update.update.items.speed1 = 0;
//    msg_update.update.items.speed2 = 0;
//    msg_update.update.items.speed3 = 0;
//    msg_update.update.items.speed4 = 0;
//    msg_update.update.items.speed5 = 0;
//    msg_update.update.items.speed6 = 1;
//msg_update.update.result = 255;
//    std::cout << (int)msg_update.update.result;
//    std::cout << (int)msg_update.update.path;
//    std::cout << (int)msg_update.update.items.speed;
//    std::cout << (int)msg_update.update.items.speed1;
//    std::cout << (int)msg_update.update.items.speed2;
//    std::cout << sizeof(msg_update.update.items.path);

//    union {
//        std::bitset<8> data = 0;
//        uint8_t result;
//    }heha;
//    std::cout<<(int)(1<<0)<<std::endl;
//    std::cout<<(int)(1<<1)<<std::endl;
//    std::cout<<(int)(1<<2)<<std::endl;
//    std::cout<<(int)(1<<3)<<std::endl;
//    std::cout<<(int)(1<<4)<<std::endl;
//    std::cout<<(int)(1<<5)<<std::endl;
//    std::cout<<(int)(1<<6)<<std::endl;
//    std::cout<<(int)(1<<7)<<std::endl;
//    heha.result = 0b10010110;
//    std::cout<<(int)heha.data[0]<<std::endl;
//    std::cout<<(int)heha.data[1]<<std::endl;
//    std::cout<<(int)heha.data[2]<<std::endl;
//    std::cout<<(int)heha.data[3]<<std::endl;
//    std::cout<<(int)heha.data[4]<<std::endl;
//    std::cout<<(int)heha.data[5]<<std::endl;
//    std::cout<<(int)heha.data[6]<<std::endl;
//    std::cout<<(int)heha.data[7]<<std::endl;
//    std::cout<< sizeof(std::bitset<8>)<<std::endl;





//    FLAGS_stderrthreshold =
//    ros::init(argc, argv, "tst_shawn");
//    ros::NodeHandle nh;





//    google::SetLogDestination(google::INFO, FLAGS_log_dir);
//    while (ros::ok()) {
//
//    }

//    ROS_WARN("Hello %s", "World");


//    ros::spin();
//    ecu_communication::DataUpload dataUpload, dataUpload1;
//    dataUpload.ID_caculate.result = 0x01020304;
//    dataUpload.data_upload_pack_one.pack[0] = 234;
//
//    dataUpload1 = dataUpload;
//
//    std::cout<< int(dataUpload1.data_upload_pack_one.cell.data_ID_one)<<std::endl;
//    std::cout<< int(dataUpload.ID_caculate.data[0])<<std::endl;
//    std::cout<< int(dataUpload.ID_caculate.data[1])<<std::endl;
//    std::cout<< int(dataUpload.ID_caculate.data[2])<<std::endl;
//    std::cout<< int(dataUpload.ID_caculate.data[3])<<std::endl;
//    dataUpload.raw_data_with_id.data_ID = 0x01020304;
//    std::cout<< int(dataUpload.raw_data_with_id.recv_raw_data[0])<<std::endl;
//    std::cout<< int(dataUpload.raw_data_with_id.recv_raw_data[1])<<std::endl;
//    std::cout<< int(dataUpload.raw_data_with_id.recv_raw_data[2])<<std::endl;
//    std::cout<< int(dataUpload.raw_data_with_id.recv_raw_data[3])<<std::endl;
//    char * a = funtst();
//    std::cout<<a;
//    ecu_communication::UDPClient udpClient;
//    udpClient.init((char *)"192.168.1.35", 8181);
//    uint8_t tstdata[5] = {0b00000010, 0x0f, 10, 0xff, 0xfe};
//    udpClient.process(tstdata, 5);
//    ecu_communication::DataDownload dataDownload;
//    dataDownload.data_download_pack_one.result_data[0] = 0;
//    dataDownload.data_download_pack_one.result_data[5] = 1;
//    dataDownload.data_download_pack_one.result_data[4] = 2;
//    dataDownload.data_download_pack_one.result_data[3] = 3;
//    dataDownload.data_download_pack_one.result_data[12] = 255;


//    std::cout<<(unsigned)dataDownload.data_download_pack_one.original_data.data_ID.result<<std::endl;
//    std::cout<<(int)dataDownload.data_download_pack_one.original_data.valid_data_mark<<std::endl;
//    std::cout<<(int)dataDownload.data_download_pack_one.original_data.valid_data_length<<std::endl;
//    std::cout<<(int)dataDownload.data_download_pack_one.original_data.data_ID.data[2]<<std::endl;
//    std::cout<<(int)dataDownload.data_download_pack_one.original_data.data_ID.data[3]<<std::endl;

//    std::cout<<(int)dataDownload.data_download_pack_one.original_data.functions_one.expect_gear_bit_one<<std::endl;
//    std::cout<<(int)dataDownload.data_download_pack_one.original_data.functions_one.reserve_bit3<<std::endl;
//
//    ecu_communication::bit_16_type tst16;
//    tst16.result = 0x01;
//    std::cout<<(int)tst16.data[0]<<std::endl;
//    std::cout<<(int)tst16.data[1]<<std::endl;
//
//    uint8_t tst8 = 0b01010101;
//    std::cout<<(int)bit_operation::get_bit(tst8, 7)<<std::endl;

//    ecu_communication::UDPServer test_server;
//    test_server.init(1234);
//    uint8_t test_array[4];
//    test_server.process(test_array, 4);
//    std::cout<<test_array;
//    test_server.init();
//    std::vector<uint8_t> test_data;
//    while (1) {
//        test_server.process(test_data, 4);
//
//    }
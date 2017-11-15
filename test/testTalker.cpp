/********************************************************************
 *   MIT License
 *  
 *   Copyright (c) 2017 Yi-ting Lei
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 ********************************************************************/

/** @file testTalker.cpp
 *  @brief Implementation of unit test for ROS node Talker
 *
 *  This file contains implementation of unit test for ROS node Talker
 *
 *  @author Yi-ting Lei
 *  @date   11/13/2017
*/

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "talkerClass.hpp"
#include "beginner_tutorials/talkerService.h"


/**
 *   @brief  Verify if the talker service exists or not
 *
 *   @param  
 *   @return 
*/

TEST(TestSuite, testTalkerServiceExist) {
    ros::NodeHandle n;

    // Register client to the master
    ros::ServiceClient client =
        n.serviceClient<beginner_tutorials::talkerService>("talkerService");

    // Assert service to be ready in 1 second
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1)));
}


/**
 *   @brief  Verify the talker service. Make sure it accepts update to 
 *           name and response OK
 *
 *   @param  
 *   @return 
*/
TEST(TestSuite, testTalkerServiceUpdate) {
    ros::NodeHandle n;

    // Register client with the master
    ros::ServiceClient client =
        n.serviceClient<beginner_tutorials::talkerService>("talkerService");

    beginner_tutorials::talkerService::Request req;
    beginner_tutorials::talkerService::Response resp;

    // input the name
    req.name = "ytlei";

    // Call service
    EXPECT_TRUE(client.call(req, resp));

    // check response
    EXPECT_STREQ("OK", resp.resp.c_str());
}

/*
 *   @brief  unit test main process
 *  
 *   @param  
 *   @return 0 when exit success
*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "testTalker");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

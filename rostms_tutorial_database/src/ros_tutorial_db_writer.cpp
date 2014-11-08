//------------------------------------------------------------------------------
// @file   : ros_tutorial_db_writer.cpp
// @brief  : write data into rostms database
// @author : Yoonseok Pyo
// @version: Ver0.0.1 (since 2014.05.13)
// @date   : 2014.05.13
//------------------------------------------------------------------------------
//include for ROS
#include <ros/ros.h>
#include <unistd.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tms_msg_db/TmsdbStamped.h>

//include for Mysql
#include <mysql/mysql.h>

//include for std
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>

//------------------------------------------------------------------------------
using std::string;

//------------------------------------------------------------------------------
class DbWriter
{
//------------------------------------------------------------------------------
private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // ROS Topic Subscriber
  ros::Subscriber data_sub;
  // ROS Parameters:
  bool isDebug;
  // MySQL structures 
  // https://dev.mysql.com/doc/refman/5.6/en/c-api-data-structures.html
  MYSQL     *connector;
  MYSQL_RES *result;
  MYSQL_ROW row;
  // MySQL information
  string dbhost;
  string dbuser;
  string dbpass;
  string dbname;
  string dbdata[100];

//------------------------------------------------------------------------------
public:
  DbWriter() : 
    nh_priv("~"),
    dbhost("192.168.4.170"),
    dbuser("root"),
    dbpass("tmsdb"),
    dbname("rostmsdb"),
    isDebug(false)
  {
    //Init parameter
    nh_priv.param("isDebug", isDebug, isDebug);
    //Init Vicon Stream
    ROS_ASSERT(init_dbwriter());
    // Subscriber for tms_db_data topic
    data_sub = nh.subscribe("tms_db_data", 100, &DbWriter::dbWriteCallback, this);
  }  

  //----------------------------------------------------------------------------
  ~DbWriter()
  {
    ROS_ASSERT(shutdown_dbwriter());
  } 

//------------------------------------------------------------------------------
private:
  bool init_dbwriter()
  {
    printf("rostms_tutorial_db_writer : Init OK!\n");

    //Connection to a MySQL database 
    connector = mysql_init(NULL);
    if (!mysql_real_connect(connector, dbhost.c_str(), dbuser.c_str(), dbpass.c_str(), dbname.c_str(), 3306, NULL, CLIENT_MULTI_STATEMENTS)) 
    {
      fprintf(stderr, "%s\n", mysql_error(connector));
      return false;
    }

    printf("MySQL(rostmsdb) opened.\n");
    return true;
  }

  //----------------------------------------------------------------------------
  bool shutdown_dbwriter()
  {
    //Close connection
    mysql_close(connector);
    printf("MySQL(rostmsdb) closed.\n");
    return true;
  }

  //----------------------------------------------------------------------------
  // dbWriteCallback function
  void dbWriteCallback(const tms_msg_db::TmsdbStamped::ConstPtr& msg){
    int32_t msgSize;
    char select_query[1024];
    char insert_query[1024];
    char delete_query[1024];
    MYSQL_ROW tmp_row;
    std::string tmpType;
    std::string tmpName;

    msgSize = msg->tmsdb.size();
    printf("%d\n", msgSize);
    for(int32_t i=0; i<msgSize; i++) {
      // Search the type, name, etc infomation in ID table
      sprintf(select_query, "SELECT type,name FROM rostmsdb.id WHERE id=%d", msg->tmsdb[i].id);
      printf("%s\n", select_query);
      mysql_query(connector, select_query);
      result  = mysql_use_result(connector);
      row     = mysql_fetch_row(result);
      tmp_row = row;
      if(isDebug) printf("%s,%s\n",tmp_row[0],tmp_row[1]);
      tmpType = tmp_row[0];
      tmpName = tmp_row[1];
      mysql_free_result(result);

      //------------------------------------------------------------------------
      // Insert new data to rostmsdb.history_data
      sprintf(insert_query, 
        "INSERT INTO rostmsdb.data_test VALUES ('%s','%s',%d,'%s',%f,%f,%f,%f,%f,%f,%f,%f,%f,'%s',%f,%d,'%s',%d,'%s',%d,%f,%d,'%s','%s');",
          msg->tmsdb[i].time.c_str(),
          msg->tmsdb[i].type.empty()?tmpType.c_str():msg->tmsdb[i].type.c_str(),
          msg->tmsdb[i].id,
          msg->tmsdb[i].name.empty()?tmpName.c_str():msg->tmsdb[i].name.c_str(),
          msg->tmsdb[i].x,
          msg->tmsdb[i].y,
          msg->tmsdb[i].z,
          msg->tmsdb[i].rr,
          msg->tmsdb[i].rp,
          msg->tmsdb[i].ry,
          msg->tmsdb[i].offset_x,
          msg->tmsdb[i].offset_y,
          msg->tmsdb[i].offset_z,
          msg->tmsdb[i].joint.empty()?"":msg->tmsdb[i].joint.c_str(),
          msg->tmsdb[i].weight,
          msg->tmsdb[i].rfid,
          msg->tmsdb[i].etcdata.empty()?"":msg->tmsdb[i].etcdata.c_str(),
          msg->tmsdb[i].place,
          msg->tmsdb[i].extfile.empty()?"":msg->tmsdb[i].extfile.c_str(),
          msg->tmsdb[i].sensor,
          msg->tmsdb[i].probability,
          msg->tmsdb[i].state,
          msg->tmsdb[i].task.empty()?"":msg->tmsdb[i].task.c_str(),
          msg->tmsdb[i].note.empty()?"":msg->tmsdb[i].note.c_str());

      if(isDebug) printf("%s\n",insert_query);

      if (mysql_query(connector, insert_query)) {
        fprintf(stderr, "%s\n", mysql_error(connector));
        printf("Write error!\n");
      }
    }
  }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv){
  //Init ROS node
  ros::init(argc, argv, "rostms_tutorial_db_writer");
  DbWriter dw;
  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
//EOF

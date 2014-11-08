//------------------------------------------------------------------------------
// @file   : ros_tutorial_db_reader.cpp
// @brief  : read the ros-tms database
// @author : Yoonseok Pyo
// @version: Ver0.0.1 (since 2014.05.13)
// @date   : 2014.05.13
//------------------------------------------------------------------------------
//include for ROS
#include <ros/ros.h>
#include <unistd.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>

//include for Mysql
#include <mysql/mysql.h>

//include for std
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>

//------------------------------------------------------------------------------
using std::string;
using std::vector;

//------------------------------------------------------------------------------
class DbReader
{
//------------------------------------------------------------------------------
private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // ROS ServiceServer
  ros::ServiceServer dr_srv;
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
  DbReader() : 
    nh_priv("~"),
    dbhost("192.168.4.170"),
    dbuser("root"),
    dbpass("tmsdb"),
    dbname("rostmsdb"),
    isDebug(false)
  {
    //Init parameter
    nh_priv.param("dbhost",   dbhost, dbhost);
    nh_priv.param("dbuser",   dbuser, dbuser);
    nh_priv.param("dbpass",   dbpass, dbpass);
    nh_priv.param("dbname",   dbname, dbname);
    nh_priv.param("isDebug", isDebug, isDebug);
    //Init Vicon Stream
    ROS_ASSERT(init_dbreader());
    //Service Server
    dr_srv = nh_priv.advertiseService("dbreader",&DbReader::dr_srv_callback, this);    
  }  

  //----------------------------------------------------------------------------
  ~DbReader()
  {
    ROS_ASSERT(shutdown_dbreader());
  } 

//------------------------------------------------------------------------------
private:
  bool init_dbreader()
  {
    printf("tms_db_reader : Init OK!\n");

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
  bool shutdown_dbreader()
  {
    //Close connection
    mysql_close(connector);
    printf("MySQL(rostmsdb) closed.\n");
    return true;
  }

  //----------------------------------------------------------------------------
  bool dr_srv_callback(tms_msg_db::TmsdbGetData::Request& req, tms_msg_db::TmsdbGetData::Response& res)
  {
    nh_priv.getParam("isDebug", isDebug);

    tms_msg_db::Tmsdb tmpDbdata;
    char select_query[1024];

    //--------------------------------------------------------------------------    
    if(req.tmsdb.id != 0) {
      if(isDebug) ROS_INFO_STREAM("ID-type Request!");
    }
    else {
      if(isDebug) ROS_INFO_STREAM("Wrong Request!");
      return false; 
    }
    
    //------------------------------------------------------------------------
    // Read the DB data (person,robot,sensor,structure,space,furniture,object)
    sprintf(select_query, "SELECT * FROM rostmsdb.data_test WHERE id=%d;", req.tmsdb.id);
    if(isDebug) printf("%s\n", select_query);

    mysql_query(connector, select_query);
    result = mysql_use_result(connector);

    while ((row = mysql_fetch_row(result)) != NULL) {
      for(int32_t j=0;j<24;j++) if(isDebug) ROS_INFO("%s, ",row[j]);
      tmpDbdata.time=row[0];
      tmpDbdata.type=row[1];
      tmpDbdata.id=atoi(row[2]);
      tmpDbdata.name=row[3];
      tmpDbdata.x=atof(row[4]);
      tmpDbdata.y=atof(row[5]);
      tmpDbdata.z=atof(row[6]);
      tmpDbdata.rr=atof(row[7]);
      tmpDbdata.rp=atof(row[8]);
      tmpDbdata.ry=atof(row[9]);
      tmpDbdata.offset_x=atof(row[10]);
      tmpDbdata.offset_y=atof(row[11]);
      tmpDbdata.offset_z=atof(row[12]);
      tmpDbdata.joint=row[13];
      tmpDbdata.weight=atof(row[14]);
      tmpDbdata.rfid=atoi(row[15]);
      tmpDbdata.etcdata=row[16];
      tmpDbdata.place=atoi(row[17]);
      tmpDbdata.extfile=row[18];
      tmpDbdata.sensor=atoi(row[19]);
      tmpDbdata.probability=atof(row[20]);
      tmpDbdata.state=atoi(row[21]);
      tmpDbdata.task=row[22];
      tmpDbdata.note=row[23];
      res.tmsdb.push_back(tmpDbdata);
    }

    if(mysql_num_rows(result) != 0)
      mysql_free_result(result);
  }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv){
  //Init ROS node
  ros::init(argc, argv, "rostms_tutorial_db_reader");
  DbReader dr;
  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
//EOF

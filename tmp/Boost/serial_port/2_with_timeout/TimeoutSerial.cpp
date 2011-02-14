/*
 * File:   TimeoutSerial.cpp
 * Author: Terraneo Federico
 * Distributed under the Boost Software License, Version 1.0.
 * Created on September 12, 2009, 3:47 PM
 *
 * v1.03: Fix for Mac OS X, now fully working on Mac.
 *
 * v1.02: Code cleanup, speed improvements, bug fixes.
 *
 * v1.01: Fixed a bug that caused errors while reading after a timeout.
 *
 * v1.00: First release.
 */

#include "TimeoutSerial.h"
#include <string>
#include <algorithm>
#include <iostream>
#include <boost/bind.hpp>

using namespace std;
using namespace boost;

TimeoutSerial::TimeoutSerial(): io(), port(io), timer(io),
        timeout(posix_time::seconds(0))
{
    
}

TimeoutSerial::TimeoutSerial(const std::string& devname, unsigned int baud_rate,
        asio::serial_port_base::parity opt_parity,
        asio::serial_port_base::character_size opt_csize,
        asio::serial_port_base::flow_control opt_flow,
        asio::serial_port_base::stop_bits opt_stop)
        : io(), port(io), timer(io), timeout(posix_time::seconds(0))
{
    open(devname,baud_rate,opt_parity,opt_csize,opt_flow,opt_stop);
}

void TimeoutSerial::open(const std::string& devname, unsigned int baud_rate,
        asio::serial_port_base::parity opt_parity,
        asio::serial_port_base::character_size opt_csize,
        asio::serial_port_base::flow_control opt_flow,
        asio::serial_port_base::stop_bits opt_stop)
{
    if(isOpen()) close();
    port.open(devname);
    port.set_option(asio::serial_port_base::baud_rate(baud_rate));
    port.set_option(opt_parity);
    port.set_option(opt_csize);
    port.set_option(opt_flow);
    port.set_option(opt_stop);
}

bool TimeoutSerial::isOpen() const
{
    return port.is_open();
}

void TimeoutSerial::close()
{
    if(isOpen()==false) return;
    port.close();
}

void TimeoutSerial::setTimeout(const posix_time::time_duration& t)
{
    timeout=t;
}

void TimeoutSerial::write(const char *data, size_t size)
{
    asio::write(port,asio::buffer(data,size));
}

void TimeoutSerial::write(const std::vector<char>& data)
{
    asio::write(port,asio::buffer(&data[0],data.size()));
}

void TimeoutSerial::writeString(const std::string& s)
{
    asio::write(port,asio::buffer(s.c_str(),s.size()));
}

void TimeoutSerial::read(char *data, size_t size)
{
    if(readData.size()>0)//If there is some data from a previous read
    {
        istream is(&readData);
        size_t toRead=min(readData.size(),size);//How many bytes to read?
        is.read(data,toRead);
        data+=toRead;
        size-=toRead;
        if(size==0) return;//If read data was enough, just return
    }
    
    setupParameters=ReadSetupParameters(data,size);
    performReadSetup(setupParameters);

    if(timeout!=posix_time::seconds(0)) //If timeout is set, start timer
    {
        timer.expires_from_now(timeout);
        timer.async_wait(boost::bind(&TimeoutSerial::timeoutExpired,this,
                asio::placeholders::error));
    }
    
    result=resultInProgress;
    bytesTransferred=0;
    for(;;)
    {
        io.run_one();
        switch(result)
        {
            case resultSuccess:
                timer.cancel();
                return;
            case resultTimeoutExpired:
                port.cancel();
                throw(timeout_exception("Timeout expired"));
            case resultError:
                timer.cancel();
                port.cancel();
                throw(boost::system::system_error(boost::system::error_code(),
                        "Error while reading"));
            //if resultInProgress remain in the loop
        }
    }
}

std::vector<char> TimeoutSerial::read(size_t size)
{
    vector<char> result(size,'\0');//Allocate a vector with the desired size
    read(&result[0],size);//Fill it with values
    return result;
}

std::string TimeoutSerial::readString(size_t size)
{
    string result(size,'\0');//Allocate a string with the desired size
    read(&result[0],size);//Fill it with values
    return result;
}

std::string TimeoutSerial::readStringUntil(const std::string& delim)
{
    // Note: if readData contains some previously read data, the call to
    // async_read_until (which is done in performReadSetup) correctly handles
    // it. If the data is enough it will also immediately call readCompleted()
    setupParameters=ReadSetupParameters(delim);
    performReadSetup(setupParameters);

    if(timeout!=posix_time::seconds(0)) //If timeout is set, start timer
    {
        timer.expires_from_now(timeout);
        timer.async_wait(boost::bind(&TimeoutSerial::timeoutExpired,this,
                asio::placeholders::error));
    }
    result=resultInProgress;
    bytesTransferred=0;
    for(;;)
    {
        io.run_one();
        switch(result)
        {
            case resultSuccess:
                {
                    timer.cancel();
                    bytesTransferred-=delim.size();//Don't count delim
                    istream is(&readData);
                    string result(bytesTransferred,'\0');//Alloc string
                    is.read(&result[0],bytesTransferred);//Fill values
                    is.ignore(delim.size());//Remove delimiter from stream
                    return result;
                }
            case resultTimeoutExpired:
                port.cancel();
                throw(timeout_exception("Timeout expired"));
            case resultError:
                timer.cancel();
                port.cancel();
                throw(boost::system::system_error(boost::system::error_code(),
                        "Error while reading"));
            //if resultInProgress remain in the loop
        }
    }
}

TimeoutSerial::~TimeoutSerial()
{

}

void TimeoutSerial::performReadSetup(const ReadSetupParameters& param)
{
    if(param.fixedSize)
    {
        asio::async_read(port,asio::buffer(param.data,param.size),boost::bind(
                &TimeoutSerial::readCompleted,this,asio::placeholders::error,
                asio::placeholders::bytes_transferred));
    } else {
        asio::async_read_until(port,readData,param.delim,boost::bind(
                &TimeoutSerial::readCompleted,this,asio::placeholders::error,
                asio::placeholders::bytes_transferred));
    }
}

void TimeoutSerial::timeoutExpired(const boost::system::error_code& error)
{
    if(result!=resultInProgress) return;
    if(error!=asio::error::operation_aborted)
    {
        result=resultTimeoutExpired;
    }
}

void TimeoutSerial::readCompleted(const boost::system::error_code& error,
        const size_t bytesTransferred)
{
    if(error)
    {
        if(error!=asio::error::operation_aborted)
        {
            #ifdef __APPLE__
            if(error.value()==45)
            {
                //Bug on OS X, it might be necessary to repeat the setup
                //http://osdir.com/ml/lib.boost.asio.user/2008-08/msg00004.html
                performReadSetup(setupParameters);
                return;
            }
            #endif //__APPLE__
            result=resultError;
        }
    } else {
        if(result!=resultInProgress) return;
        result=resultSuccess;
        this->bytesTransferred=bytesTransferred;
    }
}

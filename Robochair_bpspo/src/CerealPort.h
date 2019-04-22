 /*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, ISR University of Coimbra.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the ISR University of Coimbra nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Gonçalo Cabrita on 01/10/2010
 *********************************************************************/
 #include <stdexcept>
 #include <termios.h>
 #include <string>
 #include <vector>
 #include <stdint.h>
#include <stdio.h>
 #include <errno.h>
 #include <math.h>
 #include <poll.h>
 #include <signal.h>
 #include <fcntl.h>
 #include <iostream>
 #include <fstream>
 
 #include <boost/function.hpp>
 #include <boost/thread/thread.hpp>
 
 #define MAX_LENGTH 128


 
 namespace cereal
 {
         
	#define CEREAL_EXCEPT(except, msg, ...) \
	 { \
	     char buf[1000]; \
	     snprintf(buf, 1000, msg " (in CerealPort::%s)" , ##__VA_ARGS__, __FUNCTION__); \
	     throw except(buf); \
	 }

	#define DEF_EXCEPTION(name, parent) \
         class name : public parent { \
                 public: \
         name(const char* msg) : parent(msg) {} \
         }
   
         DEF_EXCEPTION(Exception, std::runtime_error);
 
         DEF_EXCEPTION(TimeoutException, Exception);
 
         #undef DEF_EXCEPTION
 
         class CerealPort
         {
                 public:
                 CerealPort();
                 ~CerealPort();
 
 
                 void open(const char * port_name, int baud_rate = 115200);
 
 
                 void close();
 
                 bool portOpen() { return fd_ != -1; }
 
                 int baudRate() { return baud_; }
 
 
                 int write(const char * data, int length = -1);
                 
 
             int read(char * data, int max_length, int timeout = -1);
 
 
             int readBytes(char * data, int length, int timeout = -1);
 
 
             int readLine(char * data, int length, int timeout = -1);
             
 
             bool readLine(std::string * data, int timeout = -1);
             
 
                 //TODO: int readBetween(char * data, int length, char start, char end, int timeout = -1);
             bool readBetween(std::string * data, char start, char end, int timeout = -1);
 
             int flush();
             
             //*** Stream functions ***
             
 
             bool startReadStream(boost::function<void(char*, int)> f);
             
 
             bool startReadLineStream(boost::function<void(std::string*)> f);
             
 
             bool startReadBetweenStream(boost::function<void(std::string*)> f, char start, char end);
             
                 void stopStream();
                 void pauseStream();
                 void resumeStream();
                 
                 private:
                 int fd_;
                 int baud_;
                 
                 //std::vector<char> leftovers;
                 
 
                 void readThread();
                 
 
             void readLineThread();
             
 
             void readBetweenThread(char start, char end);
                 
                 boost::thread * stream_thread_;
                 
                 boost::function<void(char*, int)> readCallback;
                 boost::function<void(std::string*)> readLineCallback;
                 boost::function<void(std::string*)> readBetweenCallback;
                 
                 bool stream_paused_;
                 bool stream_stopped_;
         };
 
 

CerealPort::CerealPort() : fd_(-1)
 {
         stream_thread_ = NULL;
 }
 
 CerealPort::~CerealPort()
 {
         if(portOpen()) close();
 }
 
 void CerealPort::open(const char * port_name, int baud_rate)
 {
         if(portOpen()) close();
   
         // Make IO non blocking. This way there are no race conditions that
         // cause blocking when a badly behaving process does a read at the same
         // time as us. Will need to switch to blocking for writes or errors
         // occur just after a replug event.
         fd_ = ::open(port_name, O_RDWR | O_NONBLOCK | O_NOCTTY);
 
         if(fd_ == -1)
         {
                 const char *extra_msg = "";
                 switch(errno)
                 {
                         case EACCES:
                         extra_msg = "You probably don't have premission to open the port for reading and writing.";
                         break;
 
                         case ENOENT:
                         extra_msg = "The requested port does not exist. Is the hokuyo connected? Was the port name misspelled?";
                         break;
                 }
                 CEREAL_EXCEPT(cereal::Exception, "Failed to open port: %s. %s (errno = %d). %s", port_name, strerror(errno), errno, extra_msg);
         }
         
         try
         {       
                 struct flock fl;
                 fl.l_type = F_WRLCK;
                 fl.l_whence = SEEK_SET;
                 fl.l_start = 0;
                 fl.l_len = 0;
                 fl.l_pid = getpid();
 
                 if(fcntl(fd_, F_SETLK, &fl) != 0)
                         CEREAL_EXCEPT(cereal::Exception, "Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", port_name, port_name);
 
                 // Settings for USB?
                 struct termios newtio;
                 tcgetattr(fd_, &newtio);
                 memset (&newtio.c_cc, 0, sizeof (newtio.c_cc));
                 newtio.c_cflag = CS8 | CLOCAL | CREAD;
                 newtio.c_iflag = IGNPAR;
                 newtio.c_oflag = 0;
                 newtio.c_lflag = 0;
                 cfsetspeed(&newtio, baud_rate);
                 baud_ = baud_rate;
 
                 // Activate new settings
                 tcflush(fd_, TCIFLUSH);
                 if(tcsetattr(fd_, TCSANOW, &newtio) < 0)
                         CEREAL_EXCEPT(cereal::Exception, "Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", port_name); 
                 usleep (200000);
         }
         catch(cereal::Exception& e)
         {
                 // These exceptions mean something failed on open and we should close
                 if(fd_ != -1) ::close(fd_);
                 fd_ = -1;
                 throw e;
         }
 }
 
 void CerealPort::close()
 {
         int retval = 0;
         
         retval = ::close(fd_);
 
         fd_ = -1;
 
         if(retval != 0)
                 CEREAL_EXCEPT(cereal::Exception, "Failed to close port properly -- error = %d: %s\n", errno, strerror(errno));
 }
 
 int CerealPort::write(const char * data, int length)
 {
         int len = length==-1 ? strlen(data) : length;
 
         // IO is currently non-blocking. This is what we want for the more cerealon read case.
         int origflags = fcntl(fd_, F_GETFL, 0);
         fcntl(fd_, F_SETFL, origflags & ~O_NONBLOCK); // TODO: @todo can we make this all work in non-blocking?
         int retval = ::write(fd_, data, len);
         fcntl(fd_, F_SETFL, origflags | O_NONBLOCK);
 
         if(retval == len) return retval;
         else CEREAL_EXCEPT(cereal::Exception, "write failed");
 }
 
 int CerealPort::read(char * buffer, int max_length, int timeout)
 {
         int ret;
 
         struct pollfd ufd[1];
         int retval;
         ufd[0].fd = fd_;
         ufd[0].events = POLLIN;
 
         if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.
 
         if((retval = poll(ufd, 1, timeout)) < 0) CEREAL_EXCEPT(cereal::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));
 
         if(retval == 0) CEREAL_EXCEPT(cereal::TimeoutException, "timeout reached");
                 
         if(ufd[0].revents & POLLERR) CEREAL_EXCEPT(cereal::Exception, "error on socket, possibly unplugged");
                 
     ret = ::read(fd_, buffer, max_length);
                 
         if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) CEREAL_EXCEPT(cereal::Exception, "read failed");
         
         return ret;
 }
 
 int CerealPort::readBytes(char * buffer, int length, int timeout)
 {
         int ret;
         int current = 0;
 
         struct pollfd ufd[1];
         int retval;
         ufd[0].fd = fd_;
         ufd[0].events = POLLIN;
 
         if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.
 
         while(current < length)
         {
                 if((retval = poll(ufd, 1, timeout)) < 0) CEREAL_EXCEPT(cereal::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));
 
                 if(retval == 0) CEREAL_EXCEPT(cereal::TimeoutException, "timeout reached");
                 
                 if(ufd[0].revents & POLLERR) CEREAL_EXCEPT(cereal::Exception, "error on socket, possibly unplugged");
                 
         ret = ::read(fd_, &buffer[current], length-current);
                 
                 if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) CEREAL_EXCEPT(cereal::Exception, "read failed");
 
                 current += ret;
         }
         return current;
 }
 
 int CerealPort::readLine(char * buffer, int length, int timeout)
 {
         int ret;
         int current = 0;
 
         struct pollfd ufd[1];
         int retval;
         ufd[0].fd = fd_;
         ufd[0].events = POLLIN;
 
         if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.
 
         while(current < length-1)
         {
                 if(current > 0)
                         if(buffer[current-1] == '\n')
                                 return current;
 
                 if((retval = poll(ufd, 1, timeout)) < 0) CEREAL_EXCEPT(cereal::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));
 
                 if(retval == 0) CEREAL_EXCEPT(cereal::TimeoutException, "timeout reached");
                 
                 if(ufd[0].revents & POLLERR) CEREAL_EXCEPT(cereal::Exception, "error on socket, possibly unplugged");
                 
         ret = ::read(fd_, &buffer[current], length-current);
                 
                 if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) CEREAL_EXCEPT(cereal::Exception, "read failed");
 
                 current += ret;
         }
         CEREAL_EXCEPT(cereal::Exception, "buffer filled without end of line being found");
 }
 
 bool CerealPort::readLine(std::string * buffer, int timeout)
 {
         int ret;
 
         struct pollfd ufd[1];
         int retval;
         ufd[0].fd = fd_;
         ufd[0].events = POLLIN;
 
         if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.
 
         buffer->clear();
         while(buffer->size() < buffer->max_size()/2)
         {
                 // Look for the end char
                 ret = buffer->find_first_of('\n');
                 if(ret > 0)
                 {
                         // If it is there clear everything after it and return
                         buffer->erase(ret+1, buffer->size()-ret-1);
                         return true;
                 }
 
                 if((retval = poll(ufd, 1, timeout)) < 0) CEREAL_EXCEPT(cereal::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));
 
                 if(retval == 0) CEREAL_EXCEPT(cereal::TimeoutException, "timeout reached");
                 
                 if(ufd[0].revents & POLLERR) CEREAL_EXCEPT(cereal::Exception, "error on socket, possibly unplugged");
                 
         char temp_buffer[128];
         ret = ::read(fd_, temp_buffer, 128);
                 
                 if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) CEREAL_EXCEPT(cereal::Exception, "read failed");
 
                 // Append the new data to the buffer
         buffer->append(temp_buffer, ret);
         }
         CEREAL_EXCEPT(cereal::Exception, "buffer filled without end of line being found");
 }
 
 bool CerealPort::readBetween(std::string * buffer, char start, char end, int timeout)
 {
         int ret;
 
         struct pollfd ufd[1];
         int retval;
         ufd[0].fd = fd_;
         ufd[0].events = POLLIN;
 
         if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.
 
         // Clear the buffer before we start
         buffer->clear();
         while(buffer->size() < buffer->max_size()/2)
         {
                 if((retval = poll(ufd, 1, timeout)) < 0) CEREAL_EXCEPT(cereal::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));
                 
                 if(retval == 0) CEREAL_EXCEPT(cereal::TimeoutException, "timeout reached");
                 
                 if(ufd[0].revents & POLLERR) CEREAL_EXCEPT(cereal::Exception, "error on socket, possibly unplugged");
                 
                 char temp_buffer[128];
                 ret = ::read(fd_, temp_buffer, 128);
         
                 if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) CEREAL_EXCEPT(cereal::Exception, "read failed");
         
                 // Append the new data to the buffer
                 buffer->append(temp_buffer, ret);
         
         // Look for the start char
         ret = buffer->find_first_of(start);
         // If it is not on the buffer, clear it
         if(ret == -1) buffer->clear();
         // If it is there, but not on the first position clear everything behind it
         else if(ret > 0) buffer->erase(0, ret);
                 
                 // Look for the end char
                 ret = buffer->find_first_of(end);
                 if(ret > 0)
                 {
                         // If it is there clear everything after it and return
                         buffer->erase(ret+1, buffer->size()-ret-1);
                         return true;
                 }
         }
         CEREAL_EXCEPT(cereal::Exception, "buffer filled without reaching end of data stream");
 }
 
 int CerealPort::flush()
 {
           int retval = tcflush(fd_, TCIOFLUSH);
           if(retval != 0) CEREAL_EXCEPT(cereal::Exception, "tcflush failed");
           
           return retval;
 }
 
 bool CerealPort::startReadStream(boost::function<void(char*, int)> f)
 {
         if(stream_thread_ != NULL) return false;
 
         stream_stopped_ = false;
         stream_paused_ = false;
         
         readCallback = f;
         
         stream_thread_ = new boost::thread(boost::bind(&CerealPort::readThread, this));
         return true;
 }
 
 void CerealPort::readThread()
 {
         char data[MAX_LENGTH];
         int ret;
 
         struct pollfd ufd[1];
         ufd[0].fd = fd_;
         ufd[0].events = POLLIN;
         
         while(!stream_stopped_)
         {
                 if(!stream_paused_)
                 {
                         if(poll(ufd, 1, 10) > 0)
                         {
                                 if(!(ufd[0].revents & POLLERR))
                                 {
                                         ret = ::read(fd_, data, MAX_LENGTH);
                                         if(ret>0)
                                         {
                                                 readCallback(data, ret);
                                         }
                                 }
                         }
                 }
         }
 }
 
 bool CerealPort::startReadLineStream(boost::function<void(std::string*)> f)
 {
         if(stream_thread_ != NULL) return false;
 
         stream_stopped_ = false;
         stream_paused_ = false;
         
         readLineCallback = f;
         
         stream_thread_ = new boost::thread(boost::bind(&CerealPort::readLineThread, this));
         return true;
 }
 
 void CerealPort::readLineThread()
 {
         std::string data;
         bool error = false;
 
         while(!stream_stopped_)
         {
                 if(!stream_paused_)
                 {
                         error = false;
                         try{ readLine(&data, 100); }
                         catch(cereal::Exception& e)
                         { 
                                 error = true;
                         }
                         
                         if(!error && data.size()>0) readLineCallback(&data);
                 }
         }
 }
 
 bool CerealPort::startReadBetweenStream(boost::function<void(std::string*)> f, char start, char end)
 {
         if(stream_thread_ != NULL) return false;
 
         stream_stopped_ = false;
         stream_paused_ = false;
         
         readBetweenCallback = f;
         
         stream_thread_ = new boost::thread(boost::bind(&CerealPort::readBetweenThread, this, start, end));
         return true;
 }
 
 void CerealPort::readBetweenThread(char start, char end)
 {
         std::string data;
         bool error = false;
 
         while(!stream_stopped_)
         {
                 if(!stream_paused_)
                 {
                         error = false;
                         try{ readBetween(&data, start, end, 100); }
                         catch(cereal::Exception& e)
                         { 
                                 error = true;
                         }
                         
                         if(!error && data.size()>0) readBetweenCallback(&data);
                 }
         }
 }
 
 void CerealPort::stopStream()
 {
         stream_stopped_ = true;
         stream_thread_->join();
         
         delete stream_thread_;
         stream_thread_ = NULL;
 }
 
 void CerealPort::pauseStream()
 {
         stream_paused_ = true;
 }
 
 void CerealPort::resumeStream()
 {
         stream_paused_ = false;
 }
 
}

 // EOF

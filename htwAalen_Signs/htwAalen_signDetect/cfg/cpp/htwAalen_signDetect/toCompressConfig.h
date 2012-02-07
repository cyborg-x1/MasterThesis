//#line 2 "/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"
// *********************************************************
// 
// File autogenerated for the htwAalen_signDetect package 
// by the dynamic_reconfigure package.
// Please do not edit.
// 
// ********************************************************/

/***********************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 ***********************************************************/

// Author: Blaise Gassend


#ifndef __htwAalen_signDetect__TOCOMPRESSCONFIG_H__
#define __htwAalen_signDetect__TOCOMPRESSCONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/config_init_mutex.h>

namespace htwAalen_signDetect
{
  class toCompressConfigStatics;
  
  class toCompressConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l, 
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      
      virtual void clamp(toCompressConfig &config, const toCompressConfig &max, const toCompressConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const toCompressConfig &config1, const toCompressConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, toCompressConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const toCompressConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, toCompressConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const toCompressConfig &config) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T toCompressConfig::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (toCompressConfig::* field);

      virtual void clamp(toCompressConfig &config, const toCompressConfig &max, const toCompressConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const toCompressConfig &config1, const toCompressConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, toCompressConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const toCompressConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, toCompressConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const toCompressConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }
    };

//#line 11 "cfg/toCompress.cfg"
      bool Measure_Performance;
//#line 12 "cfg/toCompress.cfg"
      bool MyOwn_NopenCV;
//#line 13 "cfg/toCompress.cfg"
      double double_param0;
//#line 14 "cfg/toCompress.cfg"
      double double_param1;
//#line 15 "cfg/toCompress.cfg"
      double double_param2;
//#line 16 "cfg/toCompress.cfg"
      double double_param3;
//#line 17 "cfg/toCompress.cfg"
      double double_param4;
//#line 18 "cfg/toCompress.cfg"
      double double_param5;
//#line 19 "cfg/toCompress.cfg"
      double double_param6;
//#line 20 "cfg/toCompress.cfg"
      double double_param7;
//#line 22 "cfg/toCompress.cfg"
      int maxdiv;
//#line 138 "/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        if ((*i)->fromMessage(msg, *this))
          count++;
      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("toCompressConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toMessage(msg, *this);
    }
    
    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      __toMessage__(msg, __param_descriptions__);
    }
    
    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->fromServer(nh, *this);
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const toCompressConfig &__max__ = __getMax__();
      const toCompressConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const toCompressConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const toCompressConfig &__getDefault__();
    static const toCompressConfig &__getMax__();
    static const toCompressConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    
  private:
    static const toCompressConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void toCompressConfig::ParamDescription<std::string>::clamp(toCompressConfig &config, const toCompressConfig &max, const toCompressConfig &min) const
  {
    return;
  }

  class toCompressConfigStatics
  {
    friend class toCompressConfig;
    
    toCompressConfigStatics()
    {
//#line 11 "cfg/toCompress.cfg"
      __min__.Measure_Performance = 0;
//#line 11 "cfg/toCompress.cfg"
      __max__.Measure_Performance = 1;
//#line 11 "cfg/toCompress.cfg"
      __default__.Measure_Performance = 0;
//#line 11 "cfg/toCompress.cfg"
      __param_descriptions__.push_back(toCompressConfig::AbstractParamDescriptionConstPtr(new toCompressConfig::ParamDescription<bool>("Measure_Performance", "bool", 0, "Measure time needed to filter images", "", &toCompressConfig::Measure_Performance)));
//#line 12 "cfg/toCompress.cfg"
      __min__.MyOwn_NopenCV = 0;
//#line 12 "cfg/toCompress.cfg"
      __max__.MyOwn_NopenCV = 1;
//#line 12 "cfg/toCompress.cfg"
      __default__.MyOwn_NopenCV = 0;
//#line 12 "cfg/toCompress.cfg"
      __param_descriptions__.push_back(toCompressConfig::AbstractParamDescriptionConstPtr(new toCompressConfig::ParamDescription<bool>("MyOwn_NopenCV", "bool", 0, "Enable Filters", "", &toCompressConfig::MyOwn_NopenCV)));
//#line 13 "cfg/toCompress.cfg"
      __min__.double_param0 = 0.0;
//#line 13 "cfg/toCompress.cfg"
      __max__.double_param0 = 10.0;
//#line 13 "cfg/toCompress.cfg"
      __default__.double_param0 = 0.01;
//#line 13 "cfg/toCompress.cfg"
      __param_descriptions__.push_back(toCompressConfig::AbstractParamDescriptionConstPtr(new toCompressConfig::ParamDescription<double>("double_param0", "double", 0, "A double parameter", "", &toCompressConfig::double_param0)));
//#line 14 "cfg/toCompress.cfg"
      __min__.double_param1 = 0.0;
//#line 14 "cfg/toCompress.cfg"
      __max__.double_param1 = 10.0;
//#line 14 "cfg/toCompress.cfg"
      __default__.double_param1 = 0.01;
//#line 14 "cfg/toCompress.cfg"
      __param_descriptions__.push_back(toCompressConfig::AbstractParamDescriptionConstPtr(new toCompressConfig::ParamDescription<double>("double_param1", "double", 0, "A double parameter", "", &toCompressConfig::double_param1)));
//#line 15 "cfg/toCompress.cfg"
      __min__.double_param2 = 0.0;
//#line 15 "cfg/toCompress.cfg"
      __max__.double_param2 = 10.0;
//#line 15 "cfg/toCompress.cfg"
      __default__.double_param2 = 0.01;
//#line 15 "cfg/toCompress.cfg"
      __param_descriptions__.push_back(toCompressConfig::AbstractParamDescriptionConstPtr(new toCompressConfig::ParamDescription<double>("double_param2", "double", 0, "A double parameter", "", &toCompressConfig::double_param2)));
//#line 16 "cfg/toCompress.cfg"
      __min__.double_param3 = 0.0;
//#line 16 "cfg/toCompress.cfg"
      __max__.double_param3 = 10.0;
//#line 16 "cfg/toCompress.cfg"
      __default__.double_param3 = 0.01;
//#line 16 "cfg/toCompress.cfg"
      __param_descriptions__.push_back(toCompressConfig::AbstractParamDescriptionConstPtr(new toCompressConfig::ParamDescription<double>("double_param3", "double", 0, "A double parameter", "", &toCompressConfig::double_param3)));
//#line 17 "cfg/toCompress.cfg"
      __min__.double_param4 = 0.0;
//#line 17 "cfg/toCompress.cfg"
      __max__.double_param4 = 10.0;
//#line 17 "cfg/toCompress.cfg"
      __default__.double_param4 = 0.01;
//#line 17 "cfg/toCompress.cfg"
      __param_descriptions__.push_back(toCompressConfig::AbstractParamDescriptionConstPtr(new toCompressConfig::ParamDescription<double>("double_param4", "double", 0, "A double parameter", "", &toCompressConfig::double_param4)));
//#line 18 "cfg/toCompress.cfg"
      __min__.double_param5 = 0.0;
//#line 18 "cfg/toCompress.cfg"
      __max__.double_param5 = 10.0;
//#line 18 "cfg/toCompress.cfg"
      __default__.double_param5 = 0.01;
//#line 18 "cfg/toCompress.cfg"
      __param_descriptions__.push_back(toCompressConfig::AbstractParamDescriptionConstPtr(new toCompressConfig::ParamDescription<double>("double_param5", "double", 0, "A double parameter", "", &toCompressConfig::double_param5)));
//#line 19 "cfg/toCompress.cfg"
      __min__.double_param6 = 0.0;
//#line 19 "cfg/toCompress.cfg"
      __max__.double_param6 = 10.0;
//#line 19 "cfg/toCompress.cfg"
      __default__.double_param6 = 0.01;
//#line 19 "cfg/toCompress.cfg"
      __param_descriptions__.push_back(toCompressConfig::AbstractParamDescriptionConstPtr(new toCompressConfig::ParamDescription<double>("double_param6", "double", 0, "A double parameter", "", &toCompressConfig::double_param6)));
//#line 20 "cfg/toCompress.cfg"
      __min__.double_param7 = 0.0;
//#line 20 "cfg/toCompress.cfg"
      __max__.double_param7 = 10.0;
//#line 20 "cfg/toCompress.cfg"
      __default__.double_param7 = 0.01;
//#line 20 "cfg/toCompress.cfg"
      __param_descriptions__.push_back(toCompressConfig::AbstractParamDescriptionConstPtr(new toCompressConfig::ParamDescription<double>("double_param7", "double", 0, "A double parameter", "", &toCompressConfig::double_param7)));
//#line 22 "cfg/toCompress.cfg"
      __min__.maxdiv = 1;
//#line 22 "cfg/toCompress.cfg"
      __max__.maxdiv = 4800000;
//#line 22 "cfg/toCompress.cfg"
      __default__.maxdiv = 100000;
//#line 22 "cfg/toCompress.cfg"
      __param_descriptions__.push_back(toCompressConfig::AbstractParamDescriptionConstPtr(new toCompressConfig::ParamDescription<int>("maxdiv", "int", 0, "maxdiv", "", &toCompressConfig::maxdiv)));
//#line 239 "/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"
    
      for (std::vector<toCompressConfig::AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        __description_message__.parameters.push_back(**i);
      __max__.__toMessage__(__description_message__.max, __param_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__); 
    }
    std::vector<toCompressConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    toCompressConfig __max__;
    toCompressConfig __min__;
    toCompressConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;
    static const toCompressConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static toCompressConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &toCompressConfig::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const toCompressConfig &toCompressConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const toCompressConfig &toCompressConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const toCompressConfig &toCompressConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<toCompressConfig::AbstractParamDescriptionConstPtr> &toCompressConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const toCompressConfigStatics *toCompressConfig::__get_statics__()
  {
    const static toCompressConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = toCompressConfigStatics::get_instance();
    
    return statics;
  }


}

#endif // __TOCOMPRESSRECONFIGURATOR_H__

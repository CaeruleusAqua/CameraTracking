//#line 2 "/opt/ros/indigo/share/dynamic_reconfigure/templates/ConfigType.h.template"
// *********************************************************
// 
// File autogenerated for the camera_pylon package 
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


#ifndef __camera_pylon__CAMERATRACKINGCONFIG_H__
#define __camera_pylon__CAMERATRACKINGCONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace camera_pylon
{
  class CameraTrackingConfigStatics;
  
  class CameraTrackingConfig
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
      
      virtual void clamp(CameraTrackingConfig &config, const CameraTrackingConfig &max, const CameraTrackingConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const CameraTrackingConfig &config1, const CameraTrackingConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, CameraTrackingConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const CameraTrackingConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, CameraTrackingConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const CameraTrackingConfig &config) const = 0;
      virtual void getValue(const CameraTrackingConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T CameraTrackingConfig::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (CameraTrackingConfig::* field);

      virtual void clamp(CameraTrackingConfig &config, const CameraTrackingConfig &max, const CameraTrackingConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const CameraTrackingConfig &config1, const CameraTrackingConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, CameraTrackingConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const CameraTrackingConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, CameraTrackingConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const CameraTrackingConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const CameraTrackingConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, CameraTrackingConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    template<class T, class PT>
    class GroupDescription : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string name, std::string type, int parent, int id, bool s, T PT::* f) : AbstractGroupDescription(name, type, parent, id, s), field(f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, CameraTrackingConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T (PT::* field);
      std::vector<CameraTrackingConfig::AbstractGroupDescriptionConstPtr> groups;
    };
    
class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(CameraTrackingConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("FixedFrameRate"==(*_i)->name){FixedFrameRate = boost::any_cast<bool>(val);}
        if("ExposureAuto"==(*_i)->name){ExposureAuto = boost::any_cast<std::string>(val);}
        if("GainAuto"==(*_i)->name){GainAuto = boost::any_cast<std::string>(val);}
        if("PixelFormat"==(*_i)->name){PixelFormat = boost::any_cast<std::string>(val);}
        if("ExposureTimeAbs"==(*_i)->name){ExposureTimeAbs = boost::any_cast<int>(val);}
        if("Gain"==(*_i)->name){Gain = boost::any_cast<int>(val);}
        if("AcquisitionMode"==(*_i)->name){AcquisitionMode = boost::any_cast<std::string>(val);}
        if("FrameRate"==(*_i)->name){FrameRate = boost::any_cast<double>(val);}
      }
    }

    bool FixedFrameRate;
std::string ExposureAuto;
std::string GainAuto;
std::string PixelFormat;
int ExposureTimeAbs;
int Gain;
std::string AcquisitionMode;
double FrameRate;

    bool state;
    std::string name;

    
}groups;



//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      bool FixedFrameRate;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      std::string ExposureAuto;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      std::string GainAuto;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      std::string PixelFormat;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      int ExposureTimeAbs;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      int Gain;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      std::string AcquisitionMode;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double FrameRate;
//#line 255 "/opt/ros/indigo/share/dynamic_reconfigure/templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("CameraTrackingConfig::__fromMessage__ called with an unexpected parameter.");
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
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }
    
    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }
    
    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const CameraTrackingConfig &__max__ = __getMax__();
      const CameraTrackingConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const CameraTrackingConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const CameraTrackingConfig &__getDefault__();
    static const CameraTrackingConfig &__getMax__();
    static const CameraTrackingConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();
    
  private:
    static const CameraTrackingConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void CameraTrackingConfig::ParamDescription<std::string>::clamp(CameraTrackingConfig &config, const CameraTrackingConfig &max, const CameraTrackingConfig &min) const
  {
    return;
  }

  class CameraTrackingConfigStatics
  {
    friend class CameraTrackingConfig;
    
    CameraTrackingConfigStatics()
    {
CameraTrackingConfig::GroupDescription<CameraTrackingConfig::DEFAULT, CameraTrackingConfig> Default("Default", "", 0, 0, true, &CameraTrackingConfig::groups);
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.FixedFrameRate = 0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.FixedFrameRate = 1;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.FixedFrameRate = 1;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<bool>("FixedFrameRate", "bool", 0, "fixed Framerate enable", "", &CameraTrackingConfig::FixedFrameRate)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<bool>("FixedFrameRate", "bool", 0, "fixed Framerate enable", "", &CameraTrackingConfig::FixedFrameRate)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.ExposureAuto = "";
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.ExposureAuto = "";
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.ExposureAuto = "Off";
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<std::string>("ExposureAuto", "str", 0, "Automatic exposure", "{'enum_description': 'Automatic Settings', 'enum': [{'srcline': 14, 'description': 'Use Manual Settings', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Off', 'ctype': 'std::string', 'type': 'str', 'name': 'Off_'}, {'srcline': 15, 'description': 'Recalc Once', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Once', 'ctype': 'std::string', 'type': 'str', 'name': 'Once'}, {'srcline': 16, 'description': 'Recalc Continually', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Continuous', 'ctype': 'std::string', 'type': 'str', 'name': 'Continuous'}]}", &CameraTrackingConfig::ExposureAuto)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<std::string>("ExposureAuto", "str", 0, "Automatic exposure", "{'enum_description': 'Automatic Settings', 'enum': [{'srcline': 14, 'description': 'Use Manual Settings', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Off', 'ctype': 'std::string', 'type': 'str', 'name': 'Off_'}, {'srcline': 15, 'description': 'Recalc Once', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Once', 'ctype': 'std::string', 'type': 'str', 'name': 'Once'}, {'srcline': 16, 'description': 'Recalc Continually', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Continuous', 'ctype': 'std::string', 'type': 'str', 'name': 'Continuous'}]}", &CameraTrackingConfig::ExposureAuto)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.GainAuto = "";
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.GainAuto = "";
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.GainAuto = "Off";
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<std::string>("GainAuto", "str", 0, "Automatic gain", "{'enum_description': 'Automatic Settings', 'enum': [{'srcline': 14, 'description': 'Use Manual Settings', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Off', 'ctype': 'std::string', 'type': 'str', 'name': 'Off_'}, {'srcline': 15, 'description': 'Recalc Once', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Once', 'ctype': 'std::string', 'type': 'str', 'name': 'Once'}, {'srcline': 16, 'description': 'Recalc Continually', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Continuous', 'ctype': 'std::string', 'type': 'str', 'name': 'Continuous'}]}", &CameraTrackingConfig::GainAuto)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<std::string>("GainAuto", "str", 0, "Automatic gain", "{'enum_description': 'Automatic Settings', 'enum': [{'srcline': 14, 'description': 'Use Manual Settings', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Off', 'ctype': 'std::string', 'type': 'str', 'name': 'Off_'}, {'srcline': 15, 'description': 'Recalc Once', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Once', 'ctype': 'std::string', 'type': 'str', 'name': 'Once'}, {'srcline': 16, 'description': 'Recalc Continually', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Continuous', 'ctype': 'std::string', 'type': 'str', 'name': 'Continuous'}]}", &CameraTrackingConfig::GainAuto)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.PixelFormat = "";
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.PixelFormat = "";
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.PixelFormat = "Bayer_RG8";
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<std::string>("PixelFormat", "str", 0, "Pixel Format", "{'enum_description': 'Pixel Format', 'enum': [{'srcline': 19, 'description': 'Color', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Bayer_RG8', 'ctype': 'std::string', 'type': 'str', 'name': 'Bayer_RG8'}, {'srcline': 20, 'description': 'Grayscale', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Mono_8', 'ctype': 'std::string', 'type': 'str', 'name': 'Mono_8'}, {'srcline': 21, 'description': 'Color', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'YUV_422', 'ctype': 'std::string', 'type': 'str', 'name': 'YUV_422'}]}", &CameraTrackingConfig::PixelFormat)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<std::string>("PixelFormat", "str", 0, "Pixel Format", "{'enum_description': 'Pixel Format', 'enum': [{'srcline': 19, 'description': 'Color', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Bayer_RG8', 'ctype': 'std::string', 'type': 'str', 'name': 'Bayer_RG8'}, {'srcline': 20, 'description': 'Grayscale', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Mono_8', 'ctype': 'std::string', 'type': 'str', 'name': 'Mono_8'}, {'srcline': 21, 'description': 'Color', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'YUV_422', 'ctype': 'std::string', 'type': 'str', 'name': 'YUV_422'}]}", &CameraTrackingConfig::PixelFormat)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.ExposureTimeAbs = 35;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.ExposureTimeAbs = 840000;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.ExposureTimeAbs = 14000;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<int>("ExposureTimeAbs", "int", 0, "Exposure time (us)", "", &CameraTrackingConfig::ExposureTimeAbs)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<int>("ExposureTimeAbs", "int", 0, "Exposure time (us)", "", &CameraTrackingConfig::ExposureTimeAbs)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.Gain = 1;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.Gain = 3;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.Gain = 1;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<int>("Gain", "int", 0, "Gain (%)", "", &CameraTrackingConfig::Gain)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<int>("Gain", "int", 0, "Gain (%)", "", &CameraTrackingConfig::Gain)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.AcquisitionMode = "";
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.AcquisitionMode = "";
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.AcquisitionMode = "Continuous";
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<std::string>("AcquisitionMode", "str", 0, "Acquisition Mode", "{'enum_description': 'AcquisitionMode', 'enum': [{'srcline': 24, 'description': 'Capture continuously upon trigger.', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Continuous', 'ctype': 'std::string', 'type': 'str', 'name': 'Continuous_'}, {'srcline': 25, 'description': 'Capture one frame upon trigger.', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'SingleFrame', 'ctype': 'std::string', 'type': 'str', 'name': 'SingleFrame'}, {'srcline': 26, 'description': 'Capture multiple frames upon trigger.', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'MultiFrame', 'ctype': 'std::string', 'type': 'str', 'name': 'MultiFrame'}]}", &CameraTrackingConfig::AcquisitionMode)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<std::string>("AcquisitionMode", "str", 0, "Acquisition Mode", "{'enum_description': 'AcquisitionMode', 'enum': [{'srcline': 24, 'description': 'Capture continuously upon trigger.', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'Continuous', 'ctype': 'std::string', 'type': 'str', 'name': 'Continuous_'}, {'srcline': 25, 'description': 'Capture one frame upon trigger.', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'SingleFrame', 'ctype': 'std::string', 'type': 'str', 'name': 'SingleFrame'}, {'srcline': 26, 'description': 'Capture multiple frames upon trigger.', 'srcfile': '/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg', 'cconsttype': 'const char * const', 'value': 'MultiFrame', 'ctype': 'std::string', 'type': 'str', 'name': 'MultiFrame'}]}", &CameraTrackingConfig::AcquisitionMode)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.FrameRate = 1.19048;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.FrameRate = 1000.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.FrameRate = 10.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<double>("FrameRate", "double", 0, "Framerate (fps)", "", &CameraTrackingConfig::FrameRate)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CameraTrackingConfig::AbstractParamDescriptionConstPtr(new CameraTrackingConfig::ParamDescription<double>("FrameRate", "double", 0, "Framerate (fps)", "", &CameraTrackingConfig::FrameRate)));
//#line 233 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.convertParams();
//#line 233 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __group_descriptions__.push_back(CameraTrackingConfig::AbstractGroupDescriptionConstPtr(new CameraTrackingConfig::GroupDescription<CameraTrackingConfig::DEFAULT, CameraTrackingConfig>(Default)));
//#line 390 "/opt/ros/indigo/share/dynamic_reconfigure/templates/ConfigType.h.template"

      for (std::vector<CameraTrackingConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__); 
    }
    std::vector<CameraTrackingConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<CameraTrackingConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    CameraTrackingConfig __max__;
    CameraTrackingConfig __min__;
    CameraTrackingConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const CameraTrackingConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static CameraTrackingConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &CameraTrackingConfig::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const CameraTrackingConfig &CameraTrackingConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const CameraTrackingConfig &CameraTrackingConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const CameraTrackingConfig &CameraTrackingConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<CameraTrackingConfig::AbstractParamDescriptionConstPtr> &CameraTrackingConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<CameraTrackingConfig::AbstractGroupDescriptionConstPtr> &CameraTrackingConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const CameraTrackingConfigStatics *CameraTrackingConfig::__get_statics__()
  {
    const static CameraTrackingConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = CameraTrackingConfigStatics::get_instance();
    
    return statics;
  }

//#line 11 "/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg"
      const char * const CameraTracking_Off = "Off";
//#line 12 "/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg"
      const char * const CameraTracking_On = "On";
//#line 14 "/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg"
      const char * const CameraTracking_Off_ = "Off";
//#line 15 "/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg"
      const char * const CameraTracking_Once = "Once";
//#line 16 "/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg"
      const char * const CameraTracking_Continuous = "Continuous";
//#line 19 "/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg"
      const char * const CameraTracking_Bayer_RG8 = "Bayer_RG8";
//#line 20 "/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg"
      const char * const CameraTracking_Mono_8 = "Mono_8";
//#line 21 "/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg"
      const char * const CameraTracking_YUV_422 = "YUV_422";
//#line 24 "/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg"
      const char * const CameraTracking_Continuous_ = "Continuous";
//#line 25 "/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg"
      const char * const CameraTracking_SingleFrame = "SingleFrame";
//#line 26 "/home/eos/Repos/CameraTracking/src/camera_pylon/cfg/CameraConfig.cfg"
      const char * const CameraTracking_MultiFrame = "MultiFrame";
}

#endif // __CAMERATRACKINGRECONFIGURATOR_H__
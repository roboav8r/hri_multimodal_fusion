#ifndef TRACKED_OBJECT_DISPLAY_H
#define TRACKED_OBJECT_DISPLAY_H

#include <boost/circular_buffer.hpp>

#include <hri_multimodal_fusion/TrackedObject.h>
#include <rviz/message_filter_display.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}


namespace multimodal_fusion_viz
{

class TrackedObjectVisual;

class TrackedObjectDisplay: public rviz::MessageFilterDisplay<hri_multimodal_fusion::TrackedObject>
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  TrackedObjectDisplay();
  virtual ~TrackedObjectDisplay();

protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  void updateColorAndAlpha();
  void updateHistoryLength();

  // Function to handle an incoming ROS message.
private:
  void processMessage( const hri_multimodal_fusion::TrackedObject::ConstPtr& msg );

  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer<boost::shared_ptr<TrackedObjectVisual> > visuals_;

  // User-editable property variables.
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::IntProperty* history_length_property_;
};


} // end namespace multimodal_fusion_viz

#endif // TRACKED_OBJECT_DISPLAY_H
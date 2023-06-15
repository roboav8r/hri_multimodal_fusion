#ifndef TRACKED_OBJECT_VISUAL_H
#define TRACKED_OBJECT_VISUAL_H

#include <hri_multimodal_fusion/TrackedObject.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Arrow;
}

namespace multimodal_fusion_viz
{
class TrackedObjectVisual
{
public:
  TrackedObjectVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  virtual ~TrackedObjectVisual();

  void setMessage( const hri_multimodal_fusion::TrackedObject::ConstPtr& msg );


  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  void setColor( float r, float g, float b, float a );

private:
  // The object implementing the actual arrow shape
  boost::shared_ptr<rviz::Arrow> acceleration_arrow_; // TODO this is the actual shape

  Ogre::SceneNode* frame_node_;

  Ogre::SceneManager* scene_manager_;
};

} // end namespace multimodal_fusion_viz

#endif // TRACKED_OBJECT_VISUAL_H
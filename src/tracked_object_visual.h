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
    class BillboardLine;
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
  // The objects implementing the actual shapes
  std::shared_ptr<rviz::Arrow> acceleration_arrow_;// = std::make_shared<rviz::Arrow>(scene_manager_, frame_node_); // TODO figure out share pointer stuff
  std::shared_ptr<rviz::BillboardLine> wireframe_;// = std::make_shared<rviz::BillboardLine>(scene_manager_, frame_node_);

  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;
};

} // end namespace multimodal_fusion_viz

#endif // TRACKED_OBJECT_VISUAL_H
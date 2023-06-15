#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>

#include "tracked_object_visual.h"

namespace multimodal_fusion_viz
{

// BEGIN_TUTORIAL
TrackedObjectVisual::TrackedObjectVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  frame_node_ = parent_node->createChildSceneNode();

  acceleration_arrow_.reset(new rviz::Arrow( scene_manager_, frame_node_ )); // TODO rename accel arrow
}

TrackedObjectVisual::~TrackedObjectVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void TrackedObjectVisual::setMessage( const hri_multimodal_fusion::TrackedObject::ConstPtr& msg )
{
  const geometry_msgs::Vector3& v = msg->twist.linear;

  // Convert the geometry_msgs::Vector3 to an Ogre::Vector3.
  Ogre::Vector3 vel( v.x, v.y, v.z );

  // Find the magnitude of the acceleration vector.
  float length = vel.length();

  // Scale the arrow's thickness in each dimension along with its length.
  Ogre::Vector3 scale( length, length, length );
  acceleration_arrow_->setScale( scale );

  // Set the orientation of the arrow to match the direction of the
  // acceleration vector.
  acceleration_arrow_->setDirection( vel );
}

// Position and orientation are passed through to the SceneNode.
void TrackedObjectVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void TrackedObjectVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

// Color is passed through to the Arrow object.
void TrackedObjectVisual::setColor( float r, float g, float b, float a )
{
  acceleration_arrow_->setColor( r, g, b, a );
}
// END_TUTORIAL

} // end namespace 
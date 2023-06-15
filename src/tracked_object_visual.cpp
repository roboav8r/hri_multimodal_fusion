#include <stdio.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include "tracked_object_visual.h"

namespace multimodal_fusion_viz
{


TrackedObjectVisual::TrackedObjectVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();

  acceleration_arrow_ = std::make_shared<rviz::Arrow>(rviz::Arrow( scene_manager_, frame_node_ )); 
  wireframe_ = std::make_shared<rviz::BillboardLine>(rviz::BillboardLine(scene_manager_, frame_node_ ));
  acceleration_arrow_.reset(new rviz::Arrow( scene_manager_, frame_node_ )); 
  wireframe_.reset(new rviz::BillboardLine(scene_manager_, frame_node_ ));

}

TrackedObjectVisual::~TrackedObjectVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void TrackedObjectVisual::setMessage( const hri_multimodal_fusion::TrackedObject::ConstPtr& msg )
{
    const geometry_msgs::Vector3& v = msg->twist.twist.linear;

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

    // Draw the wireframe
    wireframe_->setLineWidth(.03);
    wireframe_->setMaxPointsPerLine(2);
    wireframe_->setNumLines(12);

    double wScale = 0.3, hScale = 1.5, dScale = .25; // TODO update this with covariance
    double w=wScale, h=hScale, d=dScale;
    Ogre::Vector3 bottomLeft(0, -w, 0), bottomRight(0, 0, 0), topLeft(0, -w, h), topRight(0, 0, h);
    Ogre::Vector3 rear(d, 0, 0);


    std::cout << "wireframe 1 " << std::endl;
    wireframe_->addPoint(bottomLeft);          wireframe_->addPoint(bottomRight);
    wireframe_->newLine();     wireframe_->addPoint(bottomRight);         wireframe_->addPoint(topRight);
    wireframe_->newLine();     wireframe_->addPoint(topRight);            wireframe_->addPoint(topLeft);
    wireframe_->newLine();     wireframe_->addPoint(topLeft);             wireframe_->addPoint(bottomLeft);

    // Rear quad
    wireframe_->newLine();     wireframe_->addPoint(bottomLeft + rear);   wireframe_->addPoint(bottomRight + rear);
    wireframe_->newLine();     wireframe_->addPoint(bottomRight + rear);  wireframe_->addPoint(topRight + rear);
    wireframe_->newLine();     wireframe_->addPoint(topRight + rear);     wireframe_->addPoint(topLeft + rear);
    wireframe_->newLine();     wireframe_->addPoint(topLeft + rear);      wireframe_->addPoint(bottomLeft + rear);

    // Four connecting lines between front and rear
    wireframe_->newLine();     wireframe_->addPoint(bottomLeft);          wireframe_->addPoint(bottomLeft + rear);
    wireframe_->newLine();     wireframe_->addPoint(bottomRight);         wireframe_->addPoint(bottomRight + rear);
    wireframe_->newLine();     wireframe_->addPoint(topRight);            wireframe_->addPoint(topRight + rear);
    wireframe_->newLine();     wireframe_->addPoint(topLeft);             wireframe_->addPoint(topLeft + rear);

    wireframe_->setPosition(Ogre::Vector3(-w/2, w/2, -h/2));

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
  wireframe_->setColor(r, g, b, a );
}
// END_TUTORIAL

} // end namespace 
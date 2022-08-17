vsgEarth - osgEarth on the VSG Vulkan Scene Graph

vsgEarth is a prototype of
![osgEarth](https://github.com/gwaldron/osgearth) that renders using
Vulkan and the ![Vulkan Scene Graph](https://github.com/vsg-dev/VulkanSceneGraph) (VSG).
Instead of than being a complete reimplementation of osgEarth, it uses the
map layers of osgEarth as data sources for creating a VSG scene graph.

### Status:

The vsgEarth library parses osgEarth ".earth" files. It can render
multiple image layers and an elevation layer, using a simple lighting
model, from the supported osgEarth sources. The canonical ReadyMap
example works well.

A sample program, voe_globe, provides a simple-yet-familiar interface.

vsgEarth has been tested on Linux.

### Prerequisites:
* The ![Vulkan SDK](https://www.lunarg.com/vulkan-sdk/)
* ![osgEarth](https://github.com/gwaldron/osgearth)
* ![VSG](https://github.com/vsg-dev/VulkanSceneGraph)
* ![vsgXchange](https://github.com/vsg-dev/vsgXchange)
* ![osg2vsg](https://github.com/vsg-dev/osg2vsg)

These packages come with their own prerequisites, of course, so you will need to
install them too.

vsgEarth is Free Open Source Software distributed under the LGPL.

Copyright 2022 [Pelican Mapping](http://web.pelicanmapping.com/).

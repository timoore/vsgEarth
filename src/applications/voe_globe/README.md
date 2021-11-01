VOE - Prototype osgEarth with VSG scenegraph

This is a proof-of-concept of delivering osgEarth content using Vulkan
via the VSG scenegraph library. We use osgEarth libraries linked with
Open Scene Graph and OpenGL as usual, along with VSG. At run time we
don't create an OpenGL context, but we do create CPU-side OSG and
OpenGL objects for the data obtained from osgEarth layers. We then use
the vsgXchange library to translate these data into VSG scene graph
nodes using a Paged LOD approach.

Building VOE

* [Prerequisites](#prerequisites) - list of project external
  dependencies

* [Vulkan](https://vulkan.lunarg.com/) The Vulkan SDK. The most recent
  version should be fine. Do whatever is needed to get the SDK's bin
  and lib directories into your paths i.e., run the script in the SDK.

* [VSG](https://github.com/vsg-dev/VulkanSceneGraph)
* [vsgGIS](https://github.com/vsg-dev/vsgGIS)
* [vsgXchange](https://github.com/vsg-dev/vsgGIS) 

vsgXchange can be built with the assimp interchange library, but this
is currently broken and isn't necessary for VOE.

* [Build VOE](#build-voe)
Build osgEarth with the the VSGEARTH option set to ON.

* [Run VOE](#run-voe)
* Set the VSG_FILE_PATH environment variable to the osgEarth
src/applications/voe_globe source directory so that the application
can find the SPIR-V shader files.
* Run an earth file e.g. voe_globe --window 1080 720 /home/moore/graphics/osgearth/voe/tests/readymap.earth


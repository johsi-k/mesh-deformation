## Volume and Shape Preserving Mesh Deformation

Our project implements a research paper that prescribes a method of mesh deformation that preserves both volume and shape. The prescribed method preserves shape by linearly interpolating the angles of rotation between two sets of vertices, where one of the sets is manipulatable by the user. Secondly, the volume of this deformation is preserved by applying a scaling factor proportional to the previous depth of the mesh - acting as a good approximation for preserving volume. 

## Usage

The following commands are available,

Press and **hold** the following keys to,

**f** - Select fixed vertices by moving mouse over mesh (indicated by green vertices and faces) while holding this button <br>
**h** - Select handle vertices by moving mouse over mesh (indicated by blue vertices and faces) while holding this button <br>
**e** - Erase current selection for either fixed or handle vertices by moving mouse over selected parts of mesh while holding this button <br>
**q** - Perform rotation by using the mouse wheel while holding this button <br>

Press the following keys to,

**x** - Switch axis of rotation to be around the X axis <br>
**y** - Switch axis of rotation to be around the Y axis <br>
**z** - Switch axis of rotation to be around the Z axis <br>
**r** - Reset any applied mesh deformations and selections <br>
**l** - Activate an automatic loop mode, where the mesh is rotated between 180 and -180 back and forth <br>
**v** - Toggle volumetric preservation <br>
**w** - Toggle between wireframe and solid mesh rendering modes <br>

The mesh as well as vertices placed at each vertex of the mesh (indicated by small solid spheres) are rendered in the viewport.

The vertices and faces take the following colors,

Grey - Ordinary vertices or interpolated vertices <br>
Blue - Set of vertices (along with attached faces) that have been selected to be handle vertices <br>
Green - Set of vertices (along with attached faces) that have been selected to be fixed vertices <br>
Magenta - Set of vertices (along with faces) that the mouse is currently hovering over <br>

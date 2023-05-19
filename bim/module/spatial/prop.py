# VertoolBIM Add-on - OpenBIM Blender Add-on
# Copyright (C) 2020, 2021 Nainoor Patel
#
# This file is part of VertoolBIM Add-on.
#
# VertoolBIM Add-on is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# VertoolBIM Add-on is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with VertoolBIM Add-on.  If not, see <http://www.gnu.org/licenses/>.

import bpy
from blenderbim.bim.prop import StrProperty, Attribute
from blenderbim.bim.module.spatial.data import SpatialData
from bpy.types import PropertyGroup
from bpy.props import (
    PointerProperty,
    StringProperty,
    EnumProperty,
    BoolProperty,
    IntProperty,
    FloatProperty,
    FloatVectorProperty,
    CollectionProperty,
)


class SpatialElement(PropertyGroup):
    name: StringProperty(name="Name")
    long_name: StringProperty(name="Long Name")
    has_decomposition: BoolProperty(name="Has Decomposition")
    ifc_definition_id: IntProperty(name="IFC Definition ID")
    is_selected: BoolProperty(name="Is Selected")


class BIMSpatialProperties(PropertyGroup):
    containers: CollectionProperty(name="Containers", type=SpatialElement)
    active_container_index: IntProperty(name="Active Container Index")
    active_container_id: IntProperty(name="Active Container Id")


class BIMObjectSpatialProperties(PropertyGroup):
    is_editing: BoolProperty(name="Is Editing")

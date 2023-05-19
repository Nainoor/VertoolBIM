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
from . import ui, prop, operator

classes = (
    operator.EnableEditingGeoreferencing,
    operator.DisableEditingGeoreferencing,
    operator.EditGeoreferencing,
    operator.SetIfcGridNorth,
    operator.SetBlenderGridNorth,
    operator.SetIfcTrueNorth,
    operator.SetBlenderTrueNorth,
    operator.RemoveGeoreferencing,
    operator.AddGeoreferencing,
    operator.ConvertLocalToGlobal,
    operator.ConvertGlobalToLocal,
    operator.GetCursorLocation,
    operator.SetCursorLocation,
    prop.BIMGeoreferenceProperties,
    ui.BIM_PT_gis,
    ui.BIM_PT_gis_utilities,
)


def register():
    bpy.types.Scene.BIMGeoreferenceProperties = bpy.props.PointerProperty(type=prop.BIMGeoreferenceProperties)


def unregister():
    del bpy.types.Scene.BIMGeoreferenceProperties

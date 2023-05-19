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
    operator.AddType,
    operator.AssignType,
    operator.DisableEditingType,
    operator.DuplicateType,
    operator.EnableEditingType,
    operator.PurgeUnusedTypes,
    operator.RemoveType,
    operator.SelectSimilarType,
    operator.SelectType,
    operator.SelectTypeObjects,
    operator.UnassignType,
    prop.BIMTypeProperties,
    ui.BIM_PT_type,
)


def register():
    bpy.types.Object.BIMTypeProperties = bpy.props.PointerProperty(type=prop.BIMTypeProperties)


def unregister():
    del bpy.types.Object.BIMTypeProperties

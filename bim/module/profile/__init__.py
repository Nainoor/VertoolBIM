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
from . import ui, prop, operator, data

classes = (
    operator.AddProfileDef,
    operator.DisableEditingArbitraryProfile,
    operator.DisableEditingProfile,
    operator.DisableProfileEditingUI,
    operator.EditArbitraryProfile,
    operator.EditProfile,
    operator.EnableEditingArbitraryProfile,
    operator.EnableEditingProfile,
    operator.LoadProfiles,
    operator.RemoveProfileDef,
    prop.Profile,
    prop.BIMProfileProperties,
    ui.BIM_PT_profiles,
    ui.BIM_UL_profiles,
)


def register():
    bpy.types.Scene.BIMProfileProperties = bpy.props.PointerProperty(type=prop.BIMProfileProperties)


def unregister():
    del bpy.types.Scene.BIMProfileProperties
    if data.ProfileData.preview_collection:
        bpy.utils.previews.remove(data.ProfileData.preview_collection)

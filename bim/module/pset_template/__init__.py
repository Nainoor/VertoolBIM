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
    operator.AddPropEnum,
    operator.AddPropTemplate,
    operator.AddPsetFile,
    operator.AddPsetTemplate,
    operator.DeletePropEnum,
    operator.DisableEditingPropTemplate,
    operator.DisableEditingPsetTemplate,
    operator.EditPropTemplate,
    operator.EditPsetTemplate,
    operator.EnableEditingPropTemplate,
    operator.EnableEditingPsetTemplate,
    operator.RemovePropTemplate,
    operator.RemovePsetTemplate,
    operator.SavePsetTemplateFile,
    prop.PsetTemplate,
    prop.EnumerationValues,
    prop.PropTemplate,
    prop.BIMPsetTemplateProperties,
    ui.BIM_PT_pset_template,
)


def register():
    bpy.types.Scene.BIMPsetTemplateProperties = bpy.props.PointerProperty(type=prop.BIMPsetTemplateProperties)


def unregister():
    del bpy.types.Scene.BIMPsetTemplateProperties

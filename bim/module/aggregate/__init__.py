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
    operator.BIM_OT_assign_object,
    operator.BIM_OT_unassign_object,
    operator.BIM_OT_enable_editing_aggregate,
    operator.BIM_OT_disable_editing_aggregate,
    operator.BIM_OT_add_aggregate,
    operator.BIM_OT_select_parts,
    operator.BIM_OT_select_aggregate,
    operator.BIM_OT_add_part_to_object,
    prop.BIMObjectAggregateProperties,
    ui.BIM_PT_aggregate,
)


def register():
    bpy.types.Object.BIMObjectAggregateProperties = bpy.props.PointerProperty(type=prop.BIMObjectAggregateProperties)


def unregister():
    del bpy.types.Object.BIMObjectAggregateProperties

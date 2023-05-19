# VertoolBIM Add-on - OpenBIM Blender Add-on
# Copyright (C) 2021-2022 Nainoor Patel, Yassine Oualid <yassine@sigmadimensions.com>
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
import blenderbim.tool as tool
import ifcopenshell


def refresh():
    ResourceData.is_loaded = False


class ResourceData:
    data = {}
    is_loaded = False
    cost_values = {}

    @classmethod
    def load(cls):
        cls.data = {
            "total_resources": cls.total_resources(),
            "resources": cls.resources(),
            "active_resource_ids": cls.active_resource_ids(),
            "cost_values": cls.cost_values(),
        }
        cls.is_loaded = True

    @classmethod
    def total_resources(cls):
        return len(tool.Ifc.get().by_type("IfcResource"))

    @classmethod
    def resources(cls):
        results = {}
        for resource in tool.Ifc.get().by_type("IfcResource"):
            base_quantity = None
            if resource.BaseQuantity:
                base_quantity = resource.BaseQuantity.get_info()
                del base_quantity["Unit"]
            results[resource.id()] = {
                "type": resource.is_a(),
                "BaseQuantity": base_quantity,
            }
            if resource.is_a() in ["IfcLaborResource", "IfcConstructionEquipmentResource"]:
                results[resource.id()]["Productivity"] = {}
                productivity = cls.get_productivity(resource)
                if productivity:
                    results[resource.id()]["Productivity"] = {
                        "QuantityProduced": ifcopenshell.util.resource.get_quantity_produced(productivity),
                        "TimeConsumed": ifcopenshell.util.resource.get_unit_consumed(productivity),
                        "QuantityProducedName": ifcopenshell.util.resource.get_quantity_produced_name(productivity),
                    }
        return results

    @classmethod
    def get_productivity(cls, resource):
        return ifcopenshell.util.resource.get_productivity(resource, should_inherit=False)

    @classmethod
    def cost_values(cls):
        results = []
        ifc_id = bpy.context.scene.BIMResourceProperties.active_resource_id
        if not ifc_id:
            return results
        resource = tool.Ifc.get().by_id(ifc_id)
        for cost_value in resource.BaseCosts or []:
            label = "{0:.2f}".format(ifcopenshell.util.cost.calculate_applied_value(resource, cost_value))
            label += " = {}".format(ifcopenshell.util.cost.serialise_cost_value(cost_value))
            results.append({"id": cost_value.id(), "label": label})
        return results

    @classmethod
    def active_resource_ids(cls):
        obj = bpy.context.active_object
        element = tool.Ifc.get_entity(obj)
        if not element:
            return []
        results = []
        for rel in getattr(element, "HasAssignments", []) or []:
            if rel.is_a("IfcRelAssignsToResource"):
                results.append(rel.RelatingResource.id())
        return results

from pyri.webui_browser.plugins.component import PyriWebUIBrowserComponentPluginFactory
from .jog_component import register_vue_components as jog_register_vue_components

class PyriRoboticsComponentsWebUIBrowserComponentPluginFactory(PyriWebUIBrowserComponentPluginFactory):
    def get_plugin_name(self) -> str:
        return "pyri-robotics-browser"

    def register_components(self) -> None:
        jog_register_vue_components()
        

def get_webui_browser_component_factory():
    return PyriRoboticsComponentsWebUIBrowserComponentPluginFactory()
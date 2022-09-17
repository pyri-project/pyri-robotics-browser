from typing import List, Dict, Callable, Any, Tuple
from pyri.webui_browser.plugins.panel import PyriWebUIBrowserPanelInfo, PyriWebUIBrowserPanelPluginFactory
from pyri.webui_browser import PyriWebUIBrowser
from pyri.webui_browser.golden_layout import PyriGoldenLayoutPanelConfig

_panel_infos = {
    "jog": PyriWebUIBrowserPanelInfo(
        title="Jogging",
        description = "Robot jogging panel",
        panel_type = "jog",
        panel_category = "robotics",
        component_type="pyri-robotics-jog",
        priority=10000
    )
}

_panel_default_configs = {
    "jog": PyriGoldenLayoutPanelConfig(
        component_type=_panel_infos["jog"].component_type,
        panel_id = "jog",
        panel_title = "Jogging",
        closeable= False
    )
}

class PyriRoboticsPanelsWebUIBrowserPanelPluginFactory(PyriWebUIBrowserPanelPluginFactory):
    def __init__(self):
        super().__init__()

    def get_plugin_name(self) -> str:
        return "pyri-robotics-browser"

    def get_panels_infos(self) -> Dict[str,PyriWebUIBrowserPanelInfo]:
        return _panel_infos

    def get_default_panels(self, layout_config: str = "default") -> List[Tuple[PyriWebUIBrowserPanelInfo, "PyriGoldenLayoutPanelConfig"]]:
        if layout_config.lower() == "default":
            return [
                (_panel_infos["jog"], _panel_default_configs["jog"])
            ]
        else:
            return []

def get_webui_browser_panel_factory():
    return PyriRoboticsPanelsWebUIBrowserPanelPluginFactory()
# tinyxml_tools_ros

## Overview

Parse and load parameters from XML files using tinyxml_tools, but extends the DocumentHandlerXML class to handle ROS structures.

### Usage

When using `<include_xml dir="..."/>`, you can specify a ros package using `$(find my_ros_package)`, similar to how you specify it in launch files. A complete xml might look like

```
<?xml version="1.0" ?>
<Root>
    <include_xml dir="$(find tinyxml_tools)/test/xml_includes/IncludedParams.xml"/>
</Root>
```

`$(find ...)` replaces itself with the complete path to the package (turning on logger level DEBUG will print the final path used to find the xml).

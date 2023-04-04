# loco\_perf

Performance monitoring for ``loco`` modules.

## Usage

- Create a ``loco_perf::PerformanceMonitor`` instance in your controller
- Push it at the end of your measurement modules
- Call its ``addVariablesToLog()`` in your ``initialize()``

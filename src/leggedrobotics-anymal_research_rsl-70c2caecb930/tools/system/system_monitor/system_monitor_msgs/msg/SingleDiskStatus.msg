# Single disk status.

# Name.
string name

# Statistics.
# Note: 'used' and 'available' do not sum up to 'size', as Linux filesystems
# routinely reserve between 1 and 10 percent of the filesystem for the root user.
float32 size             # [GB]
float32 used             # [GB]
float32 available        # [GB]

# Usage.
float32 usage            # [-], as a fraction of the total space.
StatusCheck usage_check

# Mounting point.
string mounting_point

#!/usr/bin/env python3 
import akpub

akpub.metadata(
     	mode=None,           # Auto-detect from datatype
        datatype="Int32",  # e.g., "String", "Int32", "Image"
        node_name="num_publisher",
        topic_name="number",
        value= 10,          # Can be number, string, or image frame
        queue=10,
        delay=0.1
        # video_source=0     # Only needed for video mode
    )


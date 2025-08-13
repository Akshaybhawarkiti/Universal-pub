#!/usr/bin/env python3
import aksub

aksub.metadata(

        mode=None,           # Auto-detect from datatype
        datatype="Int32",  # e.g., "String", "Int32", "Image"
        node_name="num_subscriber",
        topic_name="number",
        queue=10,
        delay=0.2
  )

print("Received value:", aksub.received_value)


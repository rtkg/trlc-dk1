#!/bin/bash
rerun --serve-web --port 9876 &
rerun --connect rerun+http://127.0.0.1:9876/proxy

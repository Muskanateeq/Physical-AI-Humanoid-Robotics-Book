#!/bin/bash

# Stop All Services Script
# This script stops all running services

echo "🛑 Stopping Physical AI & Humanoid Robotics Platform..."
echo ""

# Check if PID files exist
if [ ! -d "logs" ]; then
    echo "⚠️  No logs directory found. Services may not be running."
    exit 1
fi

# Function to stop a service
stop_service() {
    local name=$1
    local pid_file=$2

    if [ -f "$pid_file" ]; then
        local pid=$(cat "$pid_file")
        if ps -p $pid > /dev/null 2>&1; then
            echo "🛑 Stopping $name (PID: $pid)..."
            kill $pid
            sleep 1

            # Force kill if still running
            if ps -p $pid > /dev/null 2>&1; then
                echo "⚠️  Force stopping $name..."
                kill -9 $pid
            fi

            echo "✅ $name stopped"
        else
            echo "⚠️  $name not running (PID: $pid)"
        fi
        rm "$pid_file"
    else
        echo "⚠️  No PID file for $name"
    fi
}

# Stop services
stop_service "Backend" "logs/backend.pid"
stop_service "Auth Server" "logs/auth.pid"
stop_service "Frontend" "logs/frontend.pid"

# Alternative: Kill by process name (fallback)
echo ""
echo "🔍 Checking for any remaining processes..."

pkill -f "python main.py" 2>/dev/null && echo "✅ Killed remaining backend processes"
pkill -f "node server.js" 2>/dev/null && echo "✅ Killed remaining auth processes"
pkill -f "docusaurus start" 2>/dev/null && echo "✅ Killed remaining frontend processes"

echo ""
echo "✅ All services stopped successfully!"
echo ""
echo "To restart, run: ./start-all.sh"

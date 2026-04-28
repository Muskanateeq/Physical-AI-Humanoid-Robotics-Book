#!/bin/bash

# Quick Deployment Script for Hugging Face Spaces
# This script helps you deploy your backend to Hugging Face Spaces

echo "🚀 Hugging Face Spaces Deployment Helper"
echo "=========================================="
echo ""

# Check if git is initialized
if [ ! -d .git ]; then
    echo "📦 Initializing git repository..."
    git init
    echo "✅ Git initialized"
else
    echo "✅ Git already initialized"
fi

# Get Hugging Face username
echo ""
read -p "Enter your Hugging Face username: " HF_USERNAME

# Get Space name (with default)
echo ""
read -p "Enter Space name [physical-ai-humanoid-robotics-backend]: " SPACE_NAME
SPACE_NAME=${SPACE_NAME:-physical-ai-humanoid-robotics-backend}

# Construct Hugging Face Space URL
HF_SPACE_URL="https://huggingface.co/spaces/${HF_USERNAME}/${SPACE_NAME}"
HF_GIT_URL="https://huggingface.co/spaces/${HF_USERNAME}/${SPACE_NAME}"

echo ""
echo "📍 Your Space URL will be: ${HF_SPACE_URL}"
echo "📍 Git remote URL: ${HF_GIT_URL}"
echo ""

# Check if remote already exists
if git remote | grep -q "^hf$"; then
    echo "⚠️  Remote 'hf' already exists. Removing..."
    git remote remove hf
fi

# Add Hugging Face remote
echo "🔗 Adding Hugging Face remote..."
git remote add hf "${HF_GIT_URL}"
echo "✅ Remote added"

# Create .gitignore if it doesn't exist
if [ ! -f .gitignore ]; then
    echo ""
    echo "📝 Creating .gitignore..."
    cat > .gitignore << 'EOF'
venv/
__pycache__/
*.pyc
*.pyo
*.pyd
.Python
*.so
*.egg
*.egg-info/
dist/
build/
.env
.env.local
.pytest_cache/
.coverage
htmlcov/
*.log
.DS_Store
*.backup
*.old
uv.lock
EOF
    echo "✅ .gitignore created"
fi

# Stage all files
echo ""
echo "📦 Staging files for deployment..."
git add Dockerfile requirements.txt README.md .dockerignore rag.py src/
git add DEPLOYMENT_GUIDE.md DEPLOYMENT_CHECKLIST.md 2>/dev/null || true

# Show what will be committed
echo ""
echo "📋 Files to be deployed:"
git status --short

# Commit
echo ""
read -p "Enter commit message [Deploy to Hugging Face Spaces]: " COMMIT_MSG
COMMIT_MSG=${COMMIT_MSG:-Deploy to Hugging Face Spaces}

git commit -m "${COMMIT_MSG}"
echo "✅ Changes committed"

# Push to Hugging Face
echo ""
echo "🚀 Pushing to Hugging Face Spaces..."
echo "⚠️  You may be prompted for your Hugging Face credentials"
echo ""

git push hf main --force

echo ""
echo "✅ Deployment complete!"
echo ""
echo "📍 Your backend will be available at:"
echo "   https://${HF_USERNAME}-${SPACE_NAME}.hf.space"
echo ""
echo "📚 Next steps:"
echo "   1. Go to ${HF_SPACE_URL}"
echo "   2. Click 'Settings' → 'Repository secrets'"
echo "   3. Add all environment variables (see DEPLOYMENT_GUIDE.md)"
echo "   4. Wait for build to complete (check 'Logs' tab)"
echo "   5. Test your API at /docs endpoint"
echo ""
echo "📖 For detailed instructions, see DEPLOYMENT_GUIDE.md"

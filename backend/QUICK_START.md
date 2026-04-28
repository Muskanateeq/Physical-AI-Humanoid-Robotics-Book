# Quick Reference: Hugging Face Spaces Deployment

## 🚀 Quick Deploy (3 Steps)

### 1. Create Space
Go to: https://huggingface.co/new-space
- Name: `physical-ai-humanoid-robotics-backend`
- SDK: **Docker**
- Click "Create Space"

### 2. Upload Files
Upload these files to your Space:
```
backend/
├── Dockerfile
├── requirements.txt
├── README.md
├── .dockerignore
├── rag.py
└── src/
    ├── __init__.py
    ├── main.py
    ├── api/
    │   ├── __init__.py
    │   ├── auth.py
    │   ├── chat.py
    │   └── deps.py
    ├── database/
    │   ├── __init__.py
    │   └── database.py
    ├── models/
    │   ├── __init__.py
    │   └── user.py
    ├── schemas/
    │   └── __init__.py
    └── utils/
        ├── __init__.py
        ├── jwt.py
        └── security.py
```

### 3. Set Environment Variables
In Space Settings → Repository secrets, add:

```bash
SECRET_KEY=your-secret-key
DATABASE_URL=postgresql://...
OPENROUTER_API_KEY=your-key
QDRANT_URL=https://...
QDRANT_API_KEY=your-key
BETTER_AUTH_URL=https://neurobotics-ai-book.vercel.app
BETTER_AUTH_JWKS_URL=https://neurobotics-ai-book.vercel.app/api/auth/jwks
```

**See DEPLOYMENT_GUIDE.md for complete list**

---

## 📍 Your Backend URL
```
https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space
```

---

## 🧪 Test Deployment

```bash
# Health check
curl https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space/health

# API docs
https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space/docs

# Test chat
curl -X POST https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS?"}'
```

Or run the test script:
```bash
# Update HF_SPACE_URL in test_deployment.py first
python test_deployment.py
```

---

## 🔧 Update Frontend

### File: `physical-ai-humanoid-robotics/src/config/env.ts`
```typescript
const PRODUCTION_CONFIG = {
  AUTH_SERVER_URL: '',
  BACKEND_API_URL: 'https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space',
};
```

### File: `physical-ai-humanoid-robotics/.env.local`
```bash
BACKEND_API_URL=https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space
```

---

## 📚 Documentation Files

- **DEPLOYMENT_GUIDE.md** - Complete step-by-step guide
- **DEPLOYMENT_CHECKLIST.md** - Checklist for deployment
- **README.md** - Project documentation
- **test_deployment.py** - Automated testing script

---

## ⚠️ Common Issues

### Build fails
- Check Logs tab in your Space
- Verify all files uploaded correctly

### 500 Error
- Check environment variables are set
- Verify DATABASE_URL is accessible

### CORS Error
- Add your frontend URL to CORS_ORIGINS
- Restart Space after updating env vars

---

## 💰 Cost
- **Free tier**: CPU basic (sufficient for testing)
- **Paid**: Upgrade if needed ($0.03/hour for CPU upgrade)

---

## 📞 Support
- Hugging Face Docs: https://huggingface.co/docs/hub/spaces
- Check Space Logs tab for errors
- Test locally first: `docker build -t backend . && docker run -p 7860:7860 backend`

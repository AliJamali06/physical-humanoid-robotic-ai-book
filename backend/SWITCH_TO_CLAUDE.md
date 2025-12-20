# How to Switch from Cohere to Claude

## Current Setup (Cohere)
```python
# main.py
CHAT_MODEL = "command-r7b-12-2024"

response = cohere_client.chat(
    model=CHAT_MODEL,
    message=user_message,
    preamble=system_message,
    chat_history=chat_history,
    temperature=0.7,
    max_tokens=800
)
answer = response.text
```

## New Setup (Claude via Anthropic SDK)

### 1. Install Anthropic SDK
```bash
pip install anthropic
```

### 2. Add to .env
```
ANTHROPIC_API_KEY=sk-ant-xxxxxxxxxxxxx
```

### 3. Update main.py

```python
# Add import
import anthropic

# Initialize client
anthropic_client = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))

# Replace chat generation
response = anthropic_client.messages.create(
    model="claude-3-5-sonnet-20241022",  # or claude-3-opus, claude-3-haiku
    max_tokens=800,
    temperature=0.7,
    system=system_message,
    messages=[
        {"role": "user", "content": user_message}
    ]
)

answer = response.content[0].text
```

## Cost Comparison

**Cohere (command-r7b):**
- Free tier: 1000 requests/month
- Paid: $0.15 / 1M tokens

**Claude 3.5 Sonnet:**
- Input: $3.00 / 1M tokens
- Output: $15.00 / 1M tokens
- Much better quality

**Recommendation:**
- Development: Keep Cohere (free, fast)
- Production: Switch to Claude (better answers)

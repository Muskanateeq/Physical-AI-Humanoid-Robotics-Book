---
name: ui-ux-auth
description: Create modern, animated, premium authentication UI with signup/signin forms using Tailwind CSS. Focus on slate blue/goldenrod theme, split-screen layout, smooth animations, and responsive design. Use mathematical grid background, glassmorphism forms, and circular ripple effects. No social login, external UI libraries, or heavy backend logic.
---

# UI/UX Authentication Design Skill

This skill provides guidance for creating modern, animated, premium authentication UI with signup/signin forms using Tailwind CSS.

## When to Use This Skill

Use when Claude needs to create authentication UI components for:
- Docusaurus-based websites
- Modern signup/signin forms with animations
- Slate blue/goldenrod themed interfaces
- Split-screen layouts with smooth transitions
- Mathematical grid backgrounds with glassmorphism effects
- Circular ripple button animations
- Responsive mobile/desktop designs

## Core Design Principles

### 1. Color Theme & Visual Identity
- **Primary color**: Slate Blue (`#6a5acd`) - dominant throughout design
- **Accent color**: Goldenrod (`#daa520`) - used VERY sparingly only for:
  - Button click ripple effects
  - Focus states
  - Subtle hover effects
- **Overall feel**: Minimal, elegant, futuristic, neuro-tech inspired
- **No excessive colors or gradients** - maintain clean, premium aesthetic

### 2. Background Design
- **Full screen background** - signup/signin form has no solid background
- **Mathematical grid lines**: Very subtle, low opacity, tech/AI inspired
- **No stock images, illustrations, or decorative photos**
- **Background remains visible** through form elements

### 3. Layout Structure (Split Screen)

#### Desktop Layout
```
┌─────────────────┬─────────────────┐
│                 │                 │
│  LEFT SIDE      │  RIGHT SIDE     │
│                 │                 │
│  • Large heading │  • Form inputs  │
│  • Supporting   │  • Clean labels │
│    text         │  • Modern glass-│
│  • CTA switch   │    like feel    │
│                 │                 │
└─────────────────┴─────────────────┘
```

**Left Side Content:**
- Large heading text with premium feel
- Short supporting text (AI/robotics themed)
- CTA switch button (Signin/Signup)

**Right Side Content:**
- Minimal inputs with clean spacing
- Modern glass-like feel (light transparency)
- No solid backgrounds - maintain transparency

#### Mobile Layout
- Same content as desktop
- Vertical stack instead of split
- Animations preserved for smooth transitions

## Fixed Text Content (DO NOT CHANGE)

### Signup State
- **Heading**: "Welcome to Neurobotics"
- **Button**: "Create Account"
- **Switch text**: "Already have an account? Sign In"

### Signin State
- **Heading**: "Hello Friend"
- **Button**: "Sign In"
- **Switch text**: "Don't have an account? Create Account"

## Logo Integration
- **Use only existing "Neurobotics" logo** - no new logos or placeholders
- **Position**: Top-left or top-center floating over background
- **Style**: Clean, minimal, floating feel over background

## Animation Flow Implementation

### Signup ↔ Signin Transition
- **Smooth slide animation** - no page reload
- **Premium feel** with smooth transitions
- **Ease-in-out** motion for elegant movement
- **No aggressive motion** - maintain sophisticated feel

**Behavior:**
- Signup view: Left side shows "Welcome to Neurobotics", Right side shows "Create Account" form
- Signin click: Panels smoothly slide and swap, Left shows "Hello Friend", Right shows "Sign In" form
- Create Account click: Reverse animation, panels return to original positions

### CSS Animation Classes
```css
.slide-left-enter {
  transform: translateX(0%);
  opacity: 1;
}

.slide-left-exit {
  transform: translateX(-100%);
  opacity: 0;
}

.slide-right-enter {
  transform: translateX(0%);
  opacity: 1;
}

.slide-right-exit {
  transform: translateX(100%);
  opacity: 0;
}
```

## Button Design & Micro-Interactions

### Primary Button Styling
- **Fully rounded** (pill shape) buttons
- **Slate blue** base color
- **Clean typography** with proper spacing

```html
<button class="bg-slate-600 hover:bg-slate-500 text-white font-semibold py-3 px-6 rounded-full transition-all duration-300 transform hover:scale-105 hover:shadow-lg hover:shadow-slate-500/25 relative overflow-hidden">
  Button Text
</button>
```

### Hover Effect
- Slight scale-up effect
- Soft glow/hover state
- Smooth transition between states

### Goldenrod Ripple Effect (CRITICAL)
- **Button click creates goldenrod circular ring**
- **Expands from center of button**
- **Ripple/ring effect with smooth fade-out**
- **Minimal, not flashy** - maintain premium feel
- **Only for button clicks** - not hover or other interactions

```css
.ripple-effect {
  position: absolute;
  border-radius: 50%;
  background: radial-gradient(circle, rgba(218,165,32,0.8) 0%, rgba(218,165,32,0) 70%);
  animation: ripple 0.6s linear;
  pointer-events: none;
}

@keyframes ripple {
  0% {
    transform: scale(0);
    opacity: 1;
  }
  100% {
    transform: scale(4);
    opacity: 0;
  }
}
```

## Mathematical Grid Background

```css
.mathematical-grid {
  background-image:
    linear-gradient(rgba(106, 90, 205, 0.1) 1px, transparent 1px),
    linear-gradient(90deg, rgba(106, 90, 205, 0.1) 1px, transparent 1px);
  background-size: 40px 40px;
}
```

## Glassmorphism Form Styling

```html
<div class="bg-slate-800/30 backdrop-blur-sm rounded-2xl border border-slate-700/30 p-8">
  <!-- Form content with light transparency -->
</div>
```

## Responsive Design Implementation

### Desktop (Split Screen)
```html
<div class="grid lg:grid-cols-2 gap-8">
  <!-- Left panel -->
  <!-- Right panel -->
</div>
```

### Mobile (Stacked)
```html
<div class="lg:hidden">
  <!-- Same content stacked vertically -->
</div>
```

## Complete Component Structure

```jsx
<div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 flex items-center justify-center p-4 relative overflow-hidden">
  {/* Mathematical grid background - visible throughout */}
  <div className="absolute inset-0 opacity-20" style={{
    backgroundImage: `
      linear-gradient(rgba(106, 90, 205, 0.1) 1px, transparent 1px),
      linear-gradient(90deg, rgba(106, 90, 205, 0.1) 1px, transparent 1px)
    `,
    backgroundSize: '40px 40px'
  }}></div>

  {/* Neurobotics logo floating over background */}
  <div className="absolute top-6 left-6 z-20">
    <div className="bg-gradient-to-r from-slate-600 to-slate-700 rounded-full w-12 h-12 flex items-center justify-center">
      <span className="text-white font-bold">N</span>
    </div>
  </div>

  <div className="relative z-10 w-full max-w-6xl mx-auto">
    <div className="grid lg:grid-cols-2 gap-8">
      {/* Left Panel - Animated Content */}
      <div className={clsx(
        "bg-gradient-to-br from-slate-800/30 to-slate-900/30 backdrop-blur-sm rounded-2xl p-8 flex flex-col justify-center items-center text-center border border-slate-700/30 transition-all duration-700 ease-in-out transform",
        isSignUp ? "translate-x-0 opacity-100" : "-translate-x-full opacity-0 lg:opacity-100 lg:translate-x-0"
      )}>
        {/* Animated text content */}
      </div>

      {/* Right Panel - Form */}
      <div className={clsx(
        "bg-gradient-to-br from-slate-800/30 to-slate-900/30 backdrop-blur-sm rounded-2xl p-8 border border-slate-700/30 transition-all duration-700 ease-in-out transform",
        isSignUp ? "translate-x-0 opacity-100" : "translate-x-full opacity-0 lg:opacity-100 lg:translate-x-0"
      )}>
        {/* Form content */}
      </div>
    </div>
  </div>
</div>
```

## Animation Classes for Smooth Transitions

Use Tailwind classes for smooth transitions:
- `transition-all duration-700 ease-in-out` - premium feel with longer duration
- `transform translate-x-0 opacity-100` - for visible state
- `transform -translate-x-full opacity-0` - for exit (left)
- `transform translate-x-full opacity-0` - for exit (right)

## UX Principles

- **Minimal elements** - avoid clutter
- **Proper white space** usage
- **Clear visual hierarchy**
- **Premium, modern feel** throughout
- **Light, transparent form elements** that don't overwhelm background
- **Futuristic neuro-tech aesthetic** that matches website theme

## Accessibility Considerations

- Proper ARIA labels for form elements
- Focus states for keyboard navigation
- Sufficient color contrast
- Semantic HTML structure
- Screen reader compatibility

## Implementation Checklist

- [ ] Slate blue dominant color theme with goldenrod accents
- [ ] Mathematical grid background visible throughout
- [ ] No solid backgrounds on forms - maintain transparency
- [ ] Split-screen layout for desktop
- [ ] Stacked layout for mobile
- [ ] Smooth signup/signin transitions with slide animation
- [ ] Goldenrod ripple effect on button click (critical)
- [ ] Glassmorphism form styling with light transparency
- [ ] Fixed text content exactly as specified
- [ ] Neurobotics logo in top position
- [ ] Fully rounded pill-shaped buttons
- [ ] Premium, futuristic aesthetic maintained
- [ ] Responsive design implementation
- [ ] Accessibility features included
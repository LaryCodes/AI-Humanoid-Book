# Quick Styling Reference Guide

## 🎨 Color Variables

### Using in CSS
```css
.my-element {
  color: var(--ifm-color-primary);
  background: var(--ifm-card-background);
  border-color: var(--ifm-color-emphasis-300);
}
```

### Primary Colors
- `--ifm-color-primary` - Main brand color (electric blue)
- `--ifm-color-primary-dark` - Darker shade
- `--ifm-color-primary-light` - Lighter shade

### Accent Colors
- `--ifm-color-accent-teal` - Teal accent
- `--ifm-color-accent-purple` - Purple accent
- `--ifm-color-accent-cyan` - Cyan accent

### Backgrounds
- `--ifm-background-color` - Main background
- `--ifm-background-surface-color` - Card/surface background
- `--ifm-card-background` - Card background

### Text Colors
- `--ifm-font-color-base` - Primary text
- `--ifm-font-color-secondary` - Secondary text
- `--ifm-heading-color` - Heading text

## 🎭 Utility Classes

### Animations
```jsx
<div className="animate-fade-in">Fades in</div>
<div className="animate-fade-in-up">Slides up while fading</div>
<div className="animate-scale-in">Scales in</div>
<div className="animate-pulse">Pulses continuously</div>
<div className="animate-float">Floats up and down</div>
```

### Hover Effects
```jsx
<div className="hover-lift">Lifts on hover</div>
<div className="hover-scale">Scales on hover</div>
<div className="hover-glow">Glows on hover</div>
```

### Text Effects
```jsx
<h1 className="text-gradient">Gradient text</h1>
<h1 className="gradient-text">Alternative gradient</h1>
```

### Special Effects
```jsx
<div className="glass-effect">Glass morphism</div>
<div className="shadow-glow">Glowing shadow</div>
```

## 📦 Component Patterns

### Card Component
```jsx
<div className="card hover-lift">
  <h3>Card Title</h3>
  <p>Card content</p>
</div>
```

### Button Styles
```jsx
<button className="button button--primary button--lg">
  Primary Button
</button>

<button className="button button--secondary">
  Secondary Button
</button>
```

### Feature Card
```jsx
<div className="featureCard">
  <div className="featureIconWrapper" style={{background: 'linear-gradient(135deg, #0ea5e9 0%, #14b8a6 100%)'}}>
    {/* Icon */}
  </div>
  <div className="featureContent">
    <h3>Title</h3>
    <p>Description</p>
  </div>
</div>
```

## 📝 Markdown Enhancements

### Admonitions (Callouts)
```markdown
:::note
Blue note box
:::

:::tip
Teal tip box
:::

:::info
Cyan info box
:::

:::warning
Orange warning box
:::

:::danger
Red danger box
:::

:::success
Green success box
:::
```

### Code Blocks with Title
```markdown
```python title="example.py"
def hello_world():
    print("Hello, World!")
```
```

### Highlighted Lines
```markdown
```python {2,4-6}
def example():
    # This line is highlighted
    x = 1
    # These lines
    # are also
    # highlighted
    return x
```
```

## 🎯 Layout Patterns

### Two-Column Layout
```jsx
<div className="row">
  <div className="col col--6">
    Left column
  </div>
  <div className="col col--6">
    Right column
  </div>
</div>
```

### Three-Column Layout
```jsx
<div className="row">
  <div className="col col--4">Column 1</div>
  <div className="col col--4">Column 2</div>
  <div className="col col--4">Column 3</div>
</div>
```

### Centered Container
```jsx
<div className="container">
  <div className="row">
    <div className="col col--8 col--offset-2">
      Centered content
    </div>
  </div>
</div>
```

## 🎨 Custom Gradients

### Background Gradients
```css
.my-element {
  background: linear-gradient(135deg, 
    var(--ifm-color-primary) 0%, 
    var(--ifm-color-accent-teal) 100%
  );
}
```

### Text Gradients
```css
.gradient-heading {
  background: linear-gradient(135deg, #0ea5e9 0%, #14b8a6 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}
```

## 📱 Responsive Breakpoints

```css
/* Mobile first - default styles */
.element {
  font-size: 14px;
}

/* Tablet and up (768px+) */
@media screen and (min-width: 768px) {
  .element {
    font-size: 16px;
  }
}

/* Desktop and up (996px+) */
@media screen and (min-width: 996px) {
  .element {
    font-size: 18px;
  }
}
```

## 🔧 Common Customizations

### Custom Hero Section
```jsx
<header className="hero hero--primary">
  <div className="container">
    <h1 className="hero__title gradient-text">
      Your Title
    </h1>
    <p className="hero__subtitle">
      Your subtitle
    </p>
    <div className="buttons">
      <Link className="button button--primary button--lg" to="/docs">
        Get Started
      </Link>
    </div>
  </div>
</header>
```

### Custom Card Grid
```jsx
<section className="features">
  <div className="container">
    <div className="row">
      {items.map((item, idx) => (
        <div key={idx} className="col col--4">
          <div className="card hover-lift">
            <h3>{item.title}</h3>
            <p>{item.description}</p>
          </div>
        </div>
      ))}
    </div>
  </div>
</section>
```

## 🎬 Animation Timing

- **Fast**: 150ms - For small UI changes
- **Normal**: 300ms - For most transitions
- **Slow**: 600ms - For page transitions

```css
.element {
  transition: all var(--ifm-transition-fast) ease;
}
```

## 💡 Best Practices

1. **Use CSS Variables**: Always use theme variables for colors
2. **Mobile First**: Design for mobile, enhance for desktop
3. **Consistent Spacing**: Use multiples of 0.5rem (8px)
4. **Accessible Colors**: Maintain WCAG AA contrast ratios
5. **Smooth Animations**: Keep animations under 600ms
6. **Semantic HTML**: Use proper heading hierarchy
7. **Responsive Images**: Always set max-width: 100%
8. **Test Both Themes**: Check light and dark modes

## 🚀 Performance Tips

1. Use `transform` and `opacity` for animations (GPU accelerated)
2. Avoid animating `width`, `height`, `top`, `left`
3. Use CSS modules for component-specific styles
4. Minimize use of box-shadow on large elements
5. Lazy load images and heavy components

## 📚 Resources

- [Docusaurus Documentation](https://docusaurus.io)
- [Infima CSS Framework](https://infima.dev)
- [MDX Documentation](https://mdxjs.com)
- Design System: See `DESIGN_SYSTEM.md`

# Tasks for Docusaurus Homepage Implementation: Physical AI Humanoid Robotics Course

This document outlines the granular tasks required to implement the new Docusaurus homepage, drawing details from `spec-homepage.md` and `plan-homepage.md`. Each task is designed to be a small, actionable step.

## Phase 1: Project Setup & Configuration (Verification)

1.  **Task:** Verify `docusaurus.config.js` updates.
    *   **Description:** Confirm that `title`, `tagline`, `navbar.title`, `navbar.items` (Course Modules link), `footer.copyright`, and `theme.customCss` are correctly updated as per the plan.
    *   **Status:** Pending

2.  **Task:** Verify `src/pages/markdown-page.md` deletion.
    *   **Description:** Ensure that the `markdown-page.md` file located in `src/pages/` has been successfully removed.
    *   **Status:** Pending

3.  **Task:** Verify `sidebars.js` configuration.
    *   **Description:** Confirm that `sidebars.js` correctly configures `tutorialSidebar` for auto-generated documentation, with no manual changes required.
    *   **Status:** Pending

## Phase 2: Theming & Styling (`src/css/custom.css`)

1.  **Task:** Define CSS variables for the color palette.
    *   **Description:** Add `:root` and `[data-theme='dark']` CSS variables in `src/css/custom.css` for `primary-background-color`, `accent-cyan`, `accent-magenta`, `accent-silver`, `primary-text-color`, and `secondary-text-color` as specified.
    *   **Status:** Pending

2.  **Task:** Import Google Fonts (Roboto, Open Sans).
    *   **Description:** Add `@import url()` statements for 'Roboto' (weights 700) and 'Open Sans' (weights 400, 700) at the top of `src/css/custom.css`.
    *   **Status:** Pending

3.  **Task:** Set global font families.
    *   **Description:** Apply `font-family: 'Open Sans', sans-serif;` to `body` and `font-family: 'Roboto', sans-serif;` to `h1, h2, h3, h4, h5, h6` in `src/css/custom.css`.
    *   **Status:** Pending

4.  **Task:** Implement base styles for buttons.
    *   **Description:** Define styles for `.button--primary` including the linear gradient background (`--accent-cyan`, `--accent-magenta`), text color, font-weight, and `transform` and `box-shadow` transitions for hover effects.
    *   **Status:** Pending

5.  **Task:** Create general layout utility classes.
    *   **Description:** Add CSS classes like `.container` (for max-width and centering) and `.section--padding` (for consistent vertical spacing between sections) to `src/css/custom.css`.
    *   **Status:** Pending

## Phase 3: Asset Generation & Integration

1.  **Task:** Source/Create `hero-background.jpg`.
    *   **Description:** Obtain or create a high-resolution image for `static/img/hero-background.jpg`. This image should depict a realistic, non-AI generated humanoid robot in a futuristic, advanced, and slightly abstract setting (e.g., subtle neural network overlays or data streams). Focus on photographic quality rather than illustrative.
    *   **Status:** Pending

2.  **Task:** Source/Create `overview-visual.png`.
    *   **Description:** Obtain or create a high-resolution image for `static/img/overview-visual.png`. This image should realistically show a robotic hand interacting with or assembling complex electronic components or a circuit board, conveying precision and advanced engineering.
    *   **Status:** Pending

3.  **Task:** Create `ros.svg` icon.
    *   **Description:** Design or source a simple, clean SVG icon for ROS2, depicting interconnected gears or a stylized communication network. Save as `static/img/icons/ros.svg`.
    *   **Status:** Pending

4.  **Task:** Create `simulation.svg` icon.
    *   **Description:** Design or source a simple, clean SVG icon representing simulation, such as a 3D model on a computer screen or a virtual cube. Save as `static/img/icons/simulation.svg`.
    *   **Status:** Pending

5.  **Task:** Create `brain.svg` icon.
    *   **Description:** Design or source a simple, clean SVG icon for AI/Brain, such as a stylized brain with circuitry or neural network patterns. Save as `static/img/icons/brain.svg`.
    *   **Status:** Pending

6.  **Task:** Create `vla.svg` icon.
    *   **Description:** Design or source a simple, clean SVG icon for VLA (Vision, Language, Action), combining symbols like an eye, a speech bubble, and a hand. Save as `static/img/icons/vla.svg`.
    *   **Status:** Pending

## Phase 4: Homepage Component Implementation (`src/pages/index.js`)

1.  **Task:** Create the main `Homepage` React component.
    *   **Description:** Overhaul `src/pages/index.js`. Import `React`, `Layout` from `@theme/Layout`, and `Link` from `@docusaurus/Link`. Define the `Homepage` functional component that renders the `<Layout>` and the subsequent sections.
    *   **Status:** Pending

2.  **Task:** Implement `HomepageHero` component.
    *   **Description:** Create a sub-component for the Hero section. Include the main title (`<h1>`), subtitle (`<p>`), and a `Link` component for the "Get Started Now" button. Apply inline styles or CSS classes for the background image (`hero-background.jpg`) and text styling.
    *   **Status:** Pending

3.  **Task:** Implement `CourseOverview` component.
    *   **Description:** Create a sub-component for the Course Overview section. Implement a two-column layout using Flexbox or CSS Grid. Add the heading (`<h2>`), two paragraphs of text, and an `<img>` tag pointing to `overview-visual.png`.
    *   **Status:** Pending

4.  **Task:** Implement `CourseModules` component.
    *   **Description:** Create a sub-component for the Course Modules section. Define an array of module data (title, icon path, description). Use `map` to render `ModuleCard` components in a 2x2 grid layout.
    *   **Status:** Pending

5.  **Task:** Implement `ModuleCard` component.
    *   **Description:** Create a reusable `ModuleCard` functional component that accepts `title`, `icon`, and `description` as props. Style the card with borders, background, padding, and hover effects as described in the spec. Display the icon (using an `<img>` tag or `Svg` component), title, and description.
    *   **Status:** Pending

6.  **Task:** Implement `KeyFeatures` component.
    *   **Description:** Create a sub-component for the Key Features section. Define an array of features. Render these features as an unordered list, using custom styling for the `✓` icons (either through CSS or small SVG checkmarks).
    *   **Status:** Pending

7.  **Task:** Implement `FinalCTA` component.
    *   **Description:** Create a sub-component for the final Call-to-Action section. Include the heading (`<h2>`) and a large "Start Learning for Free" button, centered.
    *   **Status:** Pending

## Phase 5: Final Assembly & Review

1.  **Task:** Assemble all components in `index.js`.
    *   **Description:** Ensure all implemented components are correctly imported and rendered in the desired order within the `Homepage` component, nested inside the `<Layout>` component.
    *   **Status:** Pending

2.  **Task:** Run Docusaurus development server.
    *   **Description:** Execute `npm run start` (or `yarn start`) in the `physical-ai-humanoid-robotics` directory to launch the local development server.
    *   **Status:** Pending

3.  **Task:** Visually inspect the new homepage.
    *   **Description:** Open the homepage in a web browser and thoroughly check for visual consistency with the design specification, correct styling, and proper layout of all sections.
    *   **Status:** Pending

4.  **Task:** Test all links and interactive elements.
    *   **Description:** Click on all CTA buttons and module links to ensure they navigate to the correct pages. Verify hover effects and any other interactive elements.
    *   **Status:** Pending

5.  **Task:** Verify responsive design.
    *   **Description:** Test the homepage on various screen sizes (e.g., by resizing the browser window or using developer tools' responsive mode) to ensure it adapts correctly and maintains usability on mobile and tablet devices.
    *   **Status:** Pending

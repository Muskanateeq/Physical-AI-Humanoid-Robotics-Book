import React from 'react';
import Layout from '@theme-original/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <BrowserOnly fallback={<div />}>
        {() => {
          const Chatbot = require('@site/src/components/Chatbot').default;
          return <Chatbot />;
        }}
      </BrowserOnly>
    </>
  );
}

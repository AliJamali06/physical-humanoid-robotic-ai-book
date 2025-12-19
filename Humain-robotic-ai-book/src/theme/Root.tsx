import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

// Wrapper component to add global features to all pages
export default function Root({ children }) {
  return (
    <>
      {children}
      <BrowserOnly>
        {() => {
          const ChatWidget = require('../components/ChatWidget').default;
          return <ChatWidget />;
        }}
      </BrowserOnly>
    </>
  );
}
